#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
image=${SITL_IMAGE:-agilicious-sitl:jazzy}
betaloop=${BETALOOP_HOME:-$HOME/betaloop}
docker_cmd=(docker)
if ! command -v docker >/dev/null; then
  echo 'Docker is not installed. See betaflight_sitl/docker/README.md.' >&2
  exit 1
fi
if ! docker info >/dev/null 2>&1; then docker_cmd=(sudo docker); fi
if [[ "${1:-}" == build ]]; then
  exec "${docker_cmd[@]}" build -t "$image" "$repo/betaflight_sitl/docker"
fi

# Keep the absolute paths used by the existing Betaloop config valid.
mapfile -t assets < <(python3 - "$betaloop" <<'PY'
import configparser
from pathlib import Path
import sys
p = configparser.ConfigParser()
p.read(Path(sys.argv[1]) / 'config.txt')
s = p['Betaloop']
print(Path(s['AeroloopGazeboHome']).expanduser().resolve())
print(Path(s['BetaflightElf']).expanduser().resolve().parent)
PY
)
if [[ ${#assets[@]} != 2 ]]; then
  echo "Cannot read $betaloop/config.txt" >&2
  exit 1
fi
args=(run --rm --init --network host --shm-size 512m --stop-timeout 30
  --name agilicious-sitl-jazzy
  --mount "type=bind,src=$repo,dst=/source,readonly"
  --mount "type=volume,src=agilicious-sitl-jazzy,dst=/workspace"
  -e "BETALOOP_HOME=$betaloop"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}"
  -e ROS_LOCALHOST_ONLY=0 -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  -e "BUILD_JOBS=${BUILD_JOBS:-4}")
[[ -t 0 && -t 1 ]] && args+=(-it)
for asset in "$betaloop" "${assets[@]}"; do
  [[ -d "$asset" ]] || { echo "Missing directory: $asset" >&2; exit 1; }
  args+=(--mount "type=bind,src=$asset,dst=$asset,readonly")
done

gui=true
for arg in "$@"; do [[ "$arg" == --no-gazebo ]] && gui=false; done
auth_file=
trap '[[ -z "$auth_file" ]] || rm -f -- "$auth_file"' EXIT
if $gui; then
  : "${DISPLAY:?Run from a desktop terminal, or pass --no-gazebo}"
  command -v xauth >/dev/null || { echo 'Install xauth on the host.' >&2; exit 1; }
  auth_file=$(mktemp /tmp/agi-sitl-xauth.XXXXXX)
  xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$auth_file" nmerge -
  [[ -s "$auth_file" ]] || { echo 'No X11 cookie; check DISPLAY and XAUTHORITY.' >&2; exit 1; }
  args+=(-e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/sitl.xauth
    -e QT_X11_NO_MITSHM=1
    --mount 'type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly'
    --mount "type=bind,src=$auth_file,dst=/tmp/sitl.xauth,readonly")
fi
[[ ! -d /dev/dri ]] || args+=(--device /dev/dri)
if [[ "${SITL_NVIDIA:-0}" == 1 ]]; then
  args+=(--gpus all -e 'NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display')
fi
if [[ "${SITL_SOFTWARE_GL:-0}" == 1 ]]; then args+=(-e LIBGL_ALWAYS_SOFTWARE=1); fi
"${docker_cmd[@]}" "${args[@]}" "$image" "$@"
