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

# Rotor plant targets.
#
# Aeroloop models each rotor as two gz-sim LiftDrag blade elements whose lift
# coefficient is cla * alpha, with alpha = a0 - atan(axial inflow / (cp *
# rotor_speed)).  Two groups follow from that:
#
#   thrust    = rho * area * cla * a0 * cp^2 * rotor_speed^2
#   zero-lift axial inflow = a0 * cp * rotor_speed
#
# so `a0` is exactly the blade's effective angle of attack at hover, which for
# a real propeller is its geometric pitch angle minus the induced inflow angle.
# Aeroloop ships a0 = 0.025 rad (1.4 deg); every real propeller sits between
# 0.09 rad (10" cruise) and 0.23 rad (5" freestyle), and at 1.4 deg thrust
# collapses under 0.8 m/s of climb, which makes the airframe uncontrollable.
#
# The other half of the fidelity problem is rotor speed.  The blade-element
# speed at hover sets the advance ratio, i.e. how far into the envelope the
# rotor is at a given airspeed.  Aeroloop's stock speed puts it at 21 m/s
# against 60 m/s for a real 5" prop, so the model reaches an advance ratio of 1
# at only 20 m/s -- past that the rotor generates lift from translation alone
# and the vertical axis stops being controllable.  Raising maxRpm fixes that,
# but only as far as the physics step can resolve blade azimuth (see
# PHYSICS_STEP_S).
SOURCE_ROTOR_A0 = 0.025
SOURCE_ROTOR_AREA = 0.2
SOURCE_ROTOR_CP = 0.084
SOURCE_ROTOR_CLA = 0.25

AIR_DENSITY = 1.2041
GRAVITY = 9.8066
# Link masses in the Aeroloop model sum to this.
MODEL_MASS_KG = 0.54

# The plugin drives each rotor joint with a velocity servo, so the reachable
# speed is vel_p_gain / (vel_p_gain + joint damping) of the commanded one.
ROTOR_SERVO_DROOP = 0.01 / (0.01 + 0.004)
# 3352 * 0.714 = 2394 rad/s reachable, four times Aeroloop's stock value and
# 68 deg of blade azimuth per 0.5 ms step -- the same azimuth resolution the
# stock model had at 2 ms.  Hover then lands near 1000 rad/s, giving a blade
# speed of 84 m/s against 60 m/s for a real 5" propeller.
PLUGIN_MAX_RPM = 3352.0
# Effective blade angle of attack at hover, matched to a 5" freestyle prop.
OVERLAY_ROTOR_A0 = 0.23
# Static thrust at full motor output, relative to the model's weight.  Real
# freestyle quadcopters run 4:1 to 8:1; the reference trajectories in
# miscellaneous/datasets need up to 3.8:1 as pure feed-forward (LOOP14_38) plus
# headroom for the tracking controller to correct on top of it.
TARGET_THRUST_TO_WEIGHT = 5.75


def _overlay_rotor_area() -> float:
    """Blade-element area that hits TARGET_THRUST_TO_WEIGHT at the chosen a0."""
    rotor_speed_max = ROTOR_SERVO_DROOP * PLUGIN_MAX_RPM
    thrust_per_rotor = (
        TARGET_THRUST_TO_WEIGHT * MODEL_MASS_KG * GRAVITY / len(BETAFLIGHT_QUADX_JOINT_ORDER)
    )
    thrust_coefficient = thrust_per_rotor / rotor_speed_max**2
    return thrust_coefficient / (
        AIR_DENSITY * SOURCE_ROTOR_CLA * OVERLAY_ROTOR_A0 * SOURCE_ROTOR_CP**2
    )


OVERLAY_ROTOR_AREA = _overlay_rotor_area()

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
# The step also caps the usable rotor speed, because LiftDrag resolves blade
# azimuth once per step.  At 2 ms and 598.6 rad/s the rotor turns 68.6 deg per
# step; holding that resolution at PLUGIN_MAX_RPM's 2394 rad/s needs 0.5 ms,
# which measures at a real-time factor of 0.999.
PHYSICS_STEP_S = 0.001


def _retune_rotor_blade_elements(model: ET.Element) -> int:
    """Rescale the rotor LiftDrag elements to keep static thrust but survive inflow."""
    elements = [
        plugin
        for plugin in model.findall("plugin")
        if plugin.get("filename") == "gz-sim-lift-drag-system"
        and (plugin.findtext("link_name") or "").startswith("rotor_")
    ]
    if not elements:
        raise RuntimeError("no rotor LiftDrag elements found in the Aeroloop model")

    for element in elements:
        a0 = element.find("a0")
        area = element.find("area")
        if a0 is None or area is None:
            raise RuntimeError("a rotor LiftDrag element is missing <a0> or <area>")
        if not math.isclose(float(a0.text or "nan"), SOURCE_ROTOR_A0, rel_tol=1e-6):
            raise RuntimeError(
                f"unexpected rotor a0 {a0.text!r}; refusing to rescale blindly"
            )
        if not math.isclose(float(area.text or "nan"), SOURCE_ROTOR_AREA, rel_tol=1e-6):
            raise RuntimeError(
                f"unexpected rotor area {area.text!r}; refusing to rescale blindly"
            )
        # cp and cla are load-bearing in the thrust and inflow expressions
        # above, so refuse to rescale if the source model moved them.
        cp = (element.findtext("cp") or "").split()
        if not cp or not math.isclose(abs(float(cp[0])), SOURCE_ROTOR_CP, rel_tol=1e-6):
            raise RuntimeError(f"unexpected rotor cp {element.findtext('cp')!r}")
        if not math.isclose(
            float(element.findtext("cla") or "nan"), SOURCE_ROTOR_CLA, rel_tol=1e-6
        ):
            raise RuntimeError(f"unexpected rotor cla {element.findtext('cla')!r}")
        a0.text = repr(OVERLAY_ROTOR_A0)
        area.text = repr(OVERLAY_ROTOR_AREA)
    return len(elements)


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

    _retune_rotor_blade_elements(model)
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
    # Cap the sim at real time.  The source world's <real_time_factor> tag is
    # gz-sim 9+ syntax and is silently ignored by gz-sim 8 (Harmonic), letting
    # the sim run faster than real time on light scenes.  The ROS 2 control
    # loop runs on ROS/sim time at 100 Hz, so an uncapped sim demands more than
    # 100 solve cycles per wall second and trips the warm-cycle gate.
    real_time_update_rate = physics.find("real_time_update_rate")
    if real_time_update_rate is None:
        real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
    real_time_update_rate.text = repr(round(1.0 / PHYSICS_STEP_S))

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
