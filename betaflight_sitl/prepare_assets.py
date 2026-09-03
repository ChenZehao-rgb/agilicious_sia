#!/usr/bin/env python3
"""Create an isolated Aeroloop model/world overlay for Agilicious SITL.

The source Aeroloop files are never modified.  The overlay fixes the motor
packet-to-joint order, makes the rotor blade elements usable outside hover, and
routes ground-truth odometry to the Agilicious outer loop.
"""

from __future__ import annotations

import argparse
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Tuple


SOURCE_MODEL_NAME = "betaloop_iris_with_standoffs"
OVERLAY_MODEL_NAME = "betaloop_iris_agilicious"
ODOM_TOPIC = "/model/iris/odometry"
JOINT_TOPIC = "/world/betaloop_demo/model/iris/joint_state"
AERODYNAMICS_TOPIC = "/model/iris/aerodynamics"
# Companion-computer IMU: the dedicated sensor the Raspberry Pi carries, as
# opposed to the flight controller's own.  See COMPANION_IMU_* below.
COMPANION_IMU_TOPIC = "/model/iris/companion_imu"

# Companion IMU noise, modelled on an ICM-42688-P at its 1 kHz output rate.
#
# The vehicle carries two physically separate IMUs in the intended hardware:
# the flight controller's, which Betaflight uses for its rate loop and
# attitude, and one on the companion computer, which dead-reckons between RTK
# fixes.  They are different parts with independent noise, so the overlay adds
# a second sensor rather than sharing one.
#
# Getting this sensor rather than MSP_RAW_IMU to do the dead reckoning is the
# whole point: MSP is request/response and served by Betaflight's ~100 Hz
# serial task, which is far too slow and jittery to integrate across the
# ~180 ms gap between RTK fixes.
COMPANION_IMU_RATE_HZ = 1000.0
COMPANION_GYRO_NOISE = 0.0035        # rad/s, sqrt(rate) * spectral density
COMPANION_GYRO_BIAS_WALK = 1.0e-5    # rad/s^2, in-run bias instability
COMPANION_ACC_NOISE = 0.030          # m/s^2
COMPANION_ACC_BIAS_WALK = 1.0e-4     # m/s^3

# Noise on the *flight controller's* IMU, which feeds Betaflight through the
# FDM packet.  Left at zero deliberately: turning it on changes the plant the
# inner-loop PID gains in betaflight_udp.yaml were tuned against, and the D
# terms in particular are free of noise cost today.  Raise these to find out
# whether that tuning survives a real gyro -- it is a worthwhile experiment,
# but it is a different one from the state-pipeline work here.
FC_GYRO_NOISE = 0.0
FC_ACC_NOISE = 0.0

# Betaflight QUADX packet order: rear-right, front-right, rear-left,
# front-left.  Aeroloop's receiver indexes rotor elements by their SDF order.
BETAFLIGHT_QUADX_JOINT_ORDER = (
    "rotor_3_joint",
    "rotor_0_joint",
    "rotor_1_joint",
    "rotor_2_joint",
)

# Qianfeng 5136 three-blade / GEPRC Mark5 airframe supplied by the user.
AIR_DENSITY = 1.2041
GRAVITY = 9.8066
MODEL_MASS_KG = 0.700
# Base-link inertia aligned with Agilib's matching quadrotor model.  The
# source Iris SDF has a much smaller base-link inertia (most notably 0.002 vs
# 0.0065 kg m^2 in roll).  Leaving it untouched makes the simulated vehicle
# rotate far faster than both MPC and the Betaflight rate tune expect once a
# trajectory asks for appreciable differential thrust.
MODEL_BASE_INERTIA_KGM2 = (0.0065, 0.0060, 0.0115)
PROP_RADIUS_M = 5.1 * 0.0254 / 2.0
PROP_PITCH_M = 3.6 * 0.0254
PROP_MASS_KG = 0.0043
PROP_MAX_RAD_S = 29280.0 * 2.0 * math.pi / 60.0
MOTOR_AXIS_M = 0.225 / (2.0 * math.sqrt(2.0))
FRAME_SIZE_M = (0.214, 0.168, 0.042)
# Cd=1.04 times the x/y/z projected rectangular areas. These values are kept
# explicit in the generated SDF so they can later be replaced by coast-down or
# wind-tunnel identification without recompiling the plugin.
BODY_CD_AREA = (
    1.04 * FRAME_SIZE_M[1] * FRAME_SIZE_M[2],
    1.04 * FRAME_SIZE_M[0] * FRAME_SIZE_M[2],
    1.04 * FRAME_SIZE_M[0] * FRAME_SIZE_M[1],
)
PLUGIN_MAX_RPM = PROP_MAX_RAD_S  # Aeroloop's historical name; units are rad/s.

# Rate at which the plugin feeds Betaflight its simulated sensors.  Betaflight
# answers every FDM packet with a motor packet, so this is effectively its loop
# rate, and it has to stay independent of PHYSICS_STEP_S: raising it to 2 kHz
# by shrinking the step made Betaflight stop emitting motor packets entirely.
FDM_PUBLISH_HZ = 500.0

# The plugin publishes odometry every physics step unless throttled.  200 Hz is
# twice the outer-loop rate, so the controller never waits on a fresh sample.
ODOM_PUBLISH_HZ = 200.0

# Physics step.  Betaflight's Gazebo guide asks for no more than 2.5 ms and
# Aeroloop's demo world ships 4 ms.  The floor is set by how fast gz-sim can
# step, which is serial: measured on an i7-14700HX it saturates near 8500
# steps/s, so 0.2 ms still holds real time while 0.1 ms only reaches a
# real-time factor of 0.84.  That is merely slower, not wrong, because the
# adapter drives its clock from the odometry's simulated-time stamp rather than
# from the wall clock.
#
# The disk-averaged BEM integrates blade azimuth internally, so the physics
# step no longer has to resolve the visual propeller rotation.  One millisecond
# retains the 1 kHz rigid-body/contact dynamics needed by the rate loop.
PHYSICS_STEP_S = 0.001


def _replace_rotor_aerodynamics(model: ET.Element) -> int:
    """Replace point LiftDrag elements with the calibrated three-blade BEM."""
    elements = [
        plugin
        for plugin in model.findall("plugin")
        if plugin.get("filename") == "gz-sim-lift-drag-system"
        and (plugin.findtext("link_name") or "").startswith("rotor_")
    ]
    if not elements:
        raise RuntimeError("no rotor LiftDrag elements found in the Aeroloop model")

    for element in elements:
        model.remove(element)

    plugin = ET.SubElement(
        model, "plugin",
        {"filename": "AgiliciousAerodynamicsPlugin",
         "name": "agilicious::aero::AgiliciousAerodynamicsPlugin"},
    )
    values = {
        "air_density": AIR_DENSITY,
        "speed_of_sound": 343.0,
        "propeller_radius": PROP_RADIUS_M,
        "propeller_pitch": PROP_PITCH_M,
        "hub_radius": 0.008,
        "chord_root": 0.017,
        "chord_tip": 0.008,
        "num_blades": 3,
        "h_force_scale": 3.0,
        "body_cd_area": " ".join(map(str, BODY_CD_AREA)),
        "wind_velocity": "0 0 0",
        "telemetry_rate": 100.0,
        "aerodynamics_rate": 200.0,
        "telemetry_topic": AERODYNAMICS_TOPIC,
    }
    for name, value in values.items():
        ET.SubElement(plugin, name).text = str(value)
    positions = {
        "rotor_0_joint": (MOTOR_AXIS_M, -MOTOR_AXIS_M, "ccw"),
        "rotor_1_joint": (-MOTOR_AXIS_M, MOTOR_AXIS_M, "ccw"),
        "rotor_2_joint": (MOTOR_AXIS_M, MOTOR_AXIS_M, "cw"),
        "rotor_3_joint": (-MOTOR_AXIS_M, -MOTOR_AXIS_M, "cw"),
    }
    for joint, (x, y, direction) in positions.items():
        rotor = ET.SubElement(plugin, "rotor")
        ET.SubElement(rotor, "joint_name").text = joint
        ET.SubElement(rotor, "link_name").text = joint.removesuffix("_joint")
        ET.SubElement(rotor, "position").text = f"{x} {y} 0"
        ET.SubElement(rotor, "direction").text = direction
    return len(elements)


def _configure_mark5_airframe(model: ET.Element) -> None:
    """Apply the supplied 700 g, 225 mm Mark5 geometry to the copied model."""
    links = {link.get("name"): link for link in model.findall("link")}
    base = links.get("base_link")
    if base is None:
        raise RuntimeError("no base_link in Aeroloop model")
    other_mass = 0.0
    for name, link in links.items():
        if name == "base_link" or name.startswith("rotor_"):
            continue
        other_mass += float(link.findtext("inertial/mass") or 0.0)
    base.find("inertial/mass").text = repr(
        MODEL_MASS_KG - other_mass - 4.0 * PROP_MASS_KG
    )
    base_inertia = base.find("inertial/inertia")
    if base_inertia is None:
        raise RuntimeError("base_link has no inertia tensor")
    for axis, value in zip(("ixx", "iyy", "izz"), MODEL_BASE_INERTIA_KGM2):
        component = base_inertia.find(axis)
        if component is None:
            raise RuntimeError(f"base_link inertia is missing {axis}")
        component.text = repr(value)
    collision_size = base.find("collision/geometry/box/size")
    if collision_size is not None:
        collision_size.text = " ".join(map(str, FRAME_SIZE_M))

    positions = {
        "rotor_0": (MOTOR_AXIS_M, -MOTOR_AXIS_M),
        "rotor_1": (-MOTOR_AXIS_M, MOTOR_AXIS_M),
        "rotor_2": (MOTOR_AXIS_M, MOTOR_AXIS_M),
        "rotor_3": (-MOTOR_AXIS_M, -MOTOR_AXIS_M),
    }
    izz = PROP_MASS_KG * PROP_RADIUS_M**2 / 3.0
    for name, (x, y) in positions.items():
        link = links.get(name)
        if link is None:
            raise RuntimeError(f"missing {name} in Aeroloop model")
        pose = (link.findtext("pose") or "").split()
        pose[:2] = [repr(x), repr(y)]
        link.find("pose").text = " ".join(pose)
        link.find("inertial/mass").text = repr(PROP_MASS_KG)
        inertia = link.find("inertial/inertia")
        inertia.find("ixx").text = repr(0.5 * izz)
        inertia.find("iyy").text = repr(0.5 * izz)
        inertia.find("izz").text = repr(izz)
        radius = link.find("collision/geometry/cylinder/radius")
        if radius is not None:
            radius.text = repr(PROP_RADIUS_M)
        joint = model.find(f"joint[@name='{name}_joint']")
        joint.find("axis/dynamics/damping").text = "1e-5"


def _add_imu_noise(sensor: ET.Element, gyro_noise: float, gyro_walk: float,
                   acc_noise: float, acc_walk: float) -> None:
    """Attach gz-sim IMU noise to `sensor`, replacing any existing block."""
    for stale in sensor.findall("imu"):
        sensor.remove(stale)
    if not (gyro_noise or gyro_walk or acc_noise or acc_walk):
        return
    imu = ET.SubElement(sensor, "imu")
    for group, stddev, walk in (
        ("angular_velocity", gyro_noise, gyro_walk),
        ("linear_acceleration", acc_noise, acc_walk),
    ):
        element = ET.SubElement(imu, group)
        for axis in ("x", "y", "z"):
            noise = ET.SubElement(ET.SubElement(element, axis), "noise",
                                  {"type": "gaussian"})
            ET.SubElement(noise, "mean").text = "0"
            ET.SubElement(noise, "stddev").text = repr(stddev)
            ET.SubElement(noise, "bias_mean").text = "0"
            ET.SubElement(noise, "dynamic_bias_stddev").text = repr(walk)


def _add_companion_imu(model: ET.Element) -> None:
    """Add the companion computer's own IMU to the airframe's base link.

    Mounted on base_link with an identity pose, so it publishes directly in the
    FLU body frame Agilicious works in.  The flight controller's imu_sensor
    sits on iris/imu_link behind a 180 degree roll -- that flip is what puts
    Betaflight in its own FRD frame, and it is the source of every sign trap in
    the MSP path.  There is no reason to inherit it here.
    """
    base = None
    for link in model.findall("link"):
        if link.get("name") == "base_link":
            base = link
            break
    if base is None:
        raise RuntimeError("no base_link in the Aeroloop model")

    for stale in [
        sensor
        for sensor in base.findall("sensor")
        if sensor.get("name") == "companion_imu"
    ]:
        base.remove(stale)

    sensor = ET.SubElement(base, "sensor",
                           {"name": "companion_imu", "type": "imu"})
    ET.SubElement(sensor, "pose").text = "0 0 0 0 0 0"
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = repr(COMPANION_IMU_RATE_HZ)
    ET.SubElement(sensor, "topic").text = COMPANION_IMU_TOPIC
    _add_imu_noise(sensor, COMPANION_GYRO_NOISE, COMPANION_GYRO_BIAS_WALK,
                   COMPANION_ACC_NOISE, COMPANION_ACC_BIAS_WALK)


def _configure_flight_controller_imu(model: ET.Element) -> None:
    """Apply FC_* noise to the sensor Betaflight reads through the plugin."""
    for link in model.findall("link"):
        for sensor in link.findall("sensor"):
            if sensor.get("name") == "imu_sensor":
                _add_imu_noise(sensor, FC_GYRO_NOISE, 0.0, FC_ACC_NOISE, 0.0)


def _write_xml(tree: ET.ElementTree, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def prepare_assets(
    aeroloop_home: Path, source_world: Path, output: Path
) -> Tuple[Path, Path]:
    source_model_dir = aeroloop_home / "models" / SOURCE_MODEL_NAME
    source_model = source_model_dir / "model.sdf"
    source_model_config = source_model_dir / "model.config"
    for required in (source_model, source_model_config, source_world):
        if not required.is_file():
            raise FileNotFoundError(f"required Aeroloop asset not found: {required}")

    model_tree = ET.parse(source_model)
    model = model_tree.getroot().find("model")
    if model is None:
        raise RuntimeError(f"no <model> in {source_model}")

    bf_plugins = [
        plugin
        for plugin in model.findall("plugin")
        if plugin.get("filename") == "BetaflightPlugin"
    ]
    if len(bf_plugins) != 1:
        raise RuntimeError(
            f"expected one BetaflightPlugin in {source_model}, got {len(bf_plugins)}"
        )
    bf_plugin = bf_plugins[0]
    # The runner builds an isolated copy whose socket wait is bounded.  The
    # plugin class/alias is unchanged; only the library filename is unique so
    # Gazebo cannot accidentally resolve the unpatched Aeroloop binary first.
    bf_plugin.set("filename", "AgiliciousBetaflightPlugin")
    rotors = bf_plugin.findall("rotor")
    rotor_by_joint = {}
    for rotor in rotors:
        joint = rotor.findtext("jointName")
        if joint:
            rotor_by_joint[joint] = rotor
    if set(rotor_by_joint) != set(BETAFLIGHT_QUADX_JOINT_ORDER):
        raise RuntimeError(
            "Aeroloop rotor joints changed; refusing an ambiguous motor reorder: "
            + ", ".join(sorted(rotor_by_joint))
        )
    for rotor in rotors:
        bf_plugin.remove(rotor)
    for motor_index, joint in enumerate(BETAFLIGHT_QUADX_JOINT_ORDER):
        rotor = rotor_by_joint[joint]
        rotor.set("id", str(motor_index))
        # With the measured 4.3 g propeller inertia and negligible joint
        # damping this gives an approximately 12 ms first-order speed response.
        vel_p_gain = rotor.find("vel_p_gain")
        if vel_p_gain is not None:
            vel_p_gain.text = "0.0005"
        # Full-throttle bench power is 907.6 W at 3066 rad/s: 0.296 N m.
        # Limit the ideal joint servo so excessive aerodynamic torque produces
        # rotor droop instead of free, non-physical power.
        for name, value in (("vel_cmd_max", "0.35"), ("vel_cmd_min", "-0.35")):
            element = rotor.find(name)
            if element is not None:
                element.text = value
        for stale in rotor.findall("maxRpm"):
            rotor.remove(stale)
        ET.SubElement(rotor, "maxRpm").text = repr(PLUGIN_MAX_RPM)
        bf_plugin.append(rotor)

    # Ground-truth odometry comes from the patched Betaflight plugin, which
    # reads the physics engine's own twist.  gz-sim's OdometryPublisher would
    # instead finite-difference the pose and report Euler angle rates in place
    # of body rates, which is only valid near hover.
    for stale in [
        plugin
        for plugin in model.findall("plugin")
        if plugin.get("name") == "gz::sim::systems::OdometryPublisher"
    ]:
        model.remove(stale)
    for stale in bf_plugin.findall("odometryTopic") + bf_plugin.findall(
        "odometryPublishFrequency"
    ):
        bf_plugin.remove(stale)
    for stale in bf_plugin.findall("fdmPublishFrequency"):
        bf_plugin.remove(stale)
    ET.SubElement(bf_plugin, "fdmPublishFrequency").text = str(FDM_PUBLISH_HZ)

    ET.SubElement(bf_plugin, "odometryTopic").text = ODOM_TOPIC
    ET.SubElement(bf_plugin, "odometryPublishFrequency").text = str(ODOM_PUBLISH_HZ)

    # The plugin polls the motor socket with a 1 ms timeout inside the physics
    # update, so a scheduling hiccup on the Betaflight side costs several
    # receives in a row.  The count is in receive attempts, which now happen at
    # FDM_PUBLISH_HZ: Aeroloop's 5 is 10 ms of tolerance, which a loaded machine
    # exceeds regularly and which then zeroes every rotor mid-flight.  Hold
    # 200 ms, still far shorter than any real link loss.
    for stale in bf_plugin.findall("connectionTimeoutMaxCount"):
        bf_plugin.remove(stale)
    ET.SubElement(bf_plugin, "connectionTimeoutMaxCount").text = str(
        round(0.200 * FDM_PUBLISH_HZ)
    )

    _configure_mark5_airframe(model)
    _replace_rotor_aerodynamics(model)
    _add_companion_imu(model)
    _configure_flight_controller_imu(model)

    joint_state_name = "gz::sim::systems::JointStatePublisher"
    for old_joint_state in [
        p for p in model.findall("plugin") if p.get("name") == joint_state_name
    ]:
        model.remove(old_joint_state)
    joint_state = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "gz-sim-joint-state-publisher-system",
            "name": joint_state_name,
        },
    )
    for joint_name in BETAFLIGHT_QUADX_JOINT_ORDER:
        ET.SubElement(joint_state, "joint_name").text = joint_name

    overlay_model_dir = output / "models" / OVERLAY_MODEL_NAME
    overlay_model = overlay_model_dir / "model.sdf"
    _write_xml(model_tree, overlay_model)
    overlay_model_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_model_config, overlay_model_dir / "model.config")

    world_tree = ET.parse(source_world)
    # Betaflight's Gazebo guide requires a physics step no larger than 2.5 ms.
    # Aeroloop's demo world currently uses 4 ms, which under-samples the fast
    # Betaflight rate loop and can make the motor / attitude loop oscillatory.
    physics = world_tree.getroot().find(".//physics")
    if physics is None:
        raise RuntimeError(f"no <physics> in {source_world}")
    max_step_size = physics.find("max_step_size")
    if max_step_size is None:
        max_step_size = ET.SubElement(physics, "max_step_size")
    max_step_size.text = repr(PHYSICS_STEP_S)

    source_uri = f"model://{SOURCE_MODEL_NAME}"
    overlay_uri = f"model://{OVERLAY_MODEL_NAME}"
    replacements = 0
    for include in world_tree.getroot().findall(".//include"):
        uri = include.find("uri")
        if uri is not None and (uri.text or "").strip() == source_uri:
            uri.text = overlay_uri
            replacements += 1
    if replacements != 1:
        raise RuntimeError(
            f"expected one {source_uri} include in {source_world}, got {replacements}"
        )
    overlay_world = output / "worlds" / "betaloop_agilicious.sdf"
    _write_xml(world_tree, overlay_world)
    return overlay_world, overlay_model


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--aeroloop-home", type=Path, required=True)
    parser.add_argument("--source-world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    world, model = prepare_assets(
        args.aeroloop_home.resolve(), args.source_world.resolve(), args.output.resolve()
    )
    print(f"world={world}")
    print(f"model={model}")
    print(f"odometry={ODOM_TOPIC}")
    print(f"joints={JOINT_TOPIC}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
