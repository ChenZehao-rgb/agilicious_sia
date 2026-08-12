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

# Betaflight QUADX packet order: rear-right, front-right, rear-left,
# front-left.  Aeroloop's receiver indexes rotor elements by their SDF order.
BETAFLIGHT_QUADX_JOINT_ORDER = (
    "rotor_3_joint",
    "rotor_0_joint",
    "rotor_1_joint",
    "rotor_2_joint",
)

# Aeroloop models each rotor as two gz-sim LiftDrag blade elements whose lift
# coefficient is cla * alpha, with alpha = a0 - climb / (cp * rotor_speed).
# The stock a0 of 0.025 rad puts the blade a hair above zero lift, so thrust
# collapses once the disc sees about 0.03 * cp * rotor_speed of axial inflow:
# measured full-throttle climb tops out at 0.83 m/s and any banked acceleration
# stalls the whole vehicle.  Raising a0 and shrinking `area` by the same factor
# leaves the static thrust and the rotor drag torque bit-identical while moving
# the zero-lift inflow out to a physically sensible ~20 m/s.  A further thrust
# scale on top of that lifts the airframe from 2.9:1 to 5.8:1 thrust-to-weight.
# The reference trajectories in miscellaneous/datasets need up to 3.8:1 as pure
# feed-forward (LOOP14_38), and a tracking controller needs headroom above the
# feed-forward to correct errors, so 2.9:1 leaves the vehicle saturated.
SOURCE_ROTOR_A0 = 0.025
SOURCE_ROTOR_AREA = 0.2
INFLOW_SENSITIVITY_RELIEF = 20.0
THRUST_SCALE = 2.0
OVERLAY_ROTOR_A0 = SOURCE_ROTOR_A0 * INFLOW_SENSITIVITY_RELIEF
OVERLAY_ROTOR_AREA = SOURCE_ROTOR_AREA * THRUST_SCALE / INFLOW_SENSITIVITY_RELIEF

# The plugin publishes odometry every physics step unless throttled.  200 Hz is
# twice the outer-loop rate, so the controller never waits on a fresh sample.
ODOM_PUBLISH_HZ = 200.0

# Physics step.  Betaflight's Gazebo guide asks for no more than 2.5 ms and
# Aeroloop's demo world ships 4 ms.  The floor is set by how fast gz-sim can
# step, which is serial: measured on an i7-14700HX it saturates near 8500
# steps/s, so 0.2 ms still holds real time while 0.1 ms only reaches a
# real-time factor of 0.84.  A factor below 1 is not merely "slower": the whole
# control stack is paced by the wall clock, so the reference trajectory would
# outrun the vehicle.
#
# The step also caps the usable rotor speed, because LiftDrag resolves blade
# azimuth once per step.  At 2 ms and 598.6 rad/s the rotor turns 68.6 deg per
# step; holding that resolution allows about 1200 rad/s at 1 ms and 2400 rad/s
# at 0.5 ms.
PHYSICS_STEP_S = 0.002


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
        a0.text = repr(OVERLAY_ROTOR_A0)
        area.text = repr(OVERLAY_ROTOR_AREA)
    return len(elements)


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
    ET.SubElement(bf_plugin, "odometryTopic").text = ODOM_TOPIC
    ET.SubElement(bf_plugin, "odometryPublishFrequency").text = str(ODOM_PUBLISH_HZ)

    # The plugin polls the motor socket with a 1 ms timeout inside the physics
    # update, so a scheduling hiccup on the Betaflight side costs several
    # receives in a row.  Aeroloop's count of 5 is 10 ms of tolerance at this
    # world's 2 ms step, which a loaded machine exceeds regularly and which
    # then zeroes every rotor mid-flight.  200 ms is still far shorter than any
    # real link loss.
    for stale in bf_plugin.findall("connectionTimeoutMaxCount"):
        bf_plugin.remove(stale)
    ET.SubElement(bf_plugin, "connectionTimeoutMaxCount").text = "100"

    _retune_rotor_blade_elements(model)

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
