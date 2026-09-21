#!/usr/bin/env bash
# Generate and validate in a temporary directory before replacing checked-in sources.
set -euo pipefail

GENERATOR_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
AGILICIOUS_ROOT="$(cd -- "${GENERATOR_DIR}/../../.." && pwd)"
ACADOS_ROOT="${ACADOS_SOURCE_DIR:-${AGILICIOUS_ROOT}/agilib/externals/acados-src}"
GENERATOR_PYTHON="${GENERATOR_PYTHON:-python3}"
export ACADOS_SOURCE_DIR="${ACADOS_ROOT}"
export LD_LIBRARY_PATH="${ACADOS_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${ACADOS_ROOT}/interfaces/acados_template${PYTHONPATH:+:${PYTHONPATH}}"

STAGING_DIR="$(mktemp -d "${TMPDIR:-/tmp}/agi_rate_mpc_codegen.XXXXXX")"
trap 'rm -rf -- "${STAGING_DIR}"' EXIT
cd -- "${STAGING_DIR}"
"${GENERATOR_PYTHON}" "${GENERATOR_DIR}/drone_model.py" --self-test

# Copy only generated source and headers, never staging binaries or object files.
"${GENERATOR_PYTHON}" - "${STAGING_DIR}/c_generated_code" "${AGILICIOUS_ROOT}" <<'PY'
import pathlib
import sys

staging = pathlib.Path(sys.argv[1])
root = pathlib.Path(sys.argv[2])
sources = root / "agilib/src/controller/mpc/acados"
headers = root / "agilib/include/agilib/controller/mpc/acados"

def copy_source(source, destination):
    # CasADi emits trailing spaces in its banner. Normalize that reproducibly
    # here, without maintaining hand-edited generated C or changing its code.
    text = "\n".join(line.rstrip() for line in source.read_text().splitlines()) + "\n"
    destination.write_text(text)

copy_source(staging / "acados_solver_drone_model.c", sources / "acados_solver_drone_model.c")
copy_source(staging / "acados_solver_drone_model.h", headers / "acados_solver_drone_model.h")
for directory in ("drone_model_model", "drone_model_cost"):
    destination = sources / directory
    destination.mkdir(exist_ok=True)
    for source in sorted((staging / directory).iterdir()):
        if source.suffix in (".c", ".h"):
            copy_source(source, destination / source.name)
print("Copied validated collective-thrust/body-rate MPC sources into", sources)
PY
