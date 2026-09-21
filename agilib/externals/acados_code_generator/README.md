# Collective-thrust/body-rate MPC generator

The checked-in solver uses an **ideal flight-controller inner loop**:

- State `x = [p_ENU(3), q_wxyz_body_to_world(4), v_ENU(3)]`, `NX=10`.
- Control `u = [c, omega_FLU(3)]`, `NU=4`, where `c = total_thrust / mass` is in m/s² and rates are in rad/s.
- Dynamics: `p_dot=v`, `q_dot=0.5*q*[0,omega]`, `v_dot=R(q)*[0,0,c]+[0,0,-9.8066]`.
- The only online parameter is the reference quaternion: `NP=4`.
- Cost residual `[p(3), attitude_error(3), v(3), c, omega(3)]`: `NY=13`, terminal `NYN=9`.
- The 20 control intervals cover 1 second. All four control components have runtime bounds; only the initial state is fixed.

The model has no rotor-force states, inertia, arm geometry, reaction-torque coefficient, aerodynamic coefficients,
motor time constant, or identified body-rate tracking delay. It assumes commanded body rate and mass-normalized
collective thrust are applied immediately. Physical flight-controller response still requires measurement and validation.
Mass remains necessary outside this predictor to convert total-thrust limits and the physical output into compatible units.

`drone_model.py` is the source of truth. Do not edit the generated C files manually.
`acados_build_copy.sh` generates and compiles in a temporary directory, checks model equations and solver hover,
and only then copies C/header files into the repository. It does not delete the existing solver before generation.
The copy step strips generated trailing whitespace reproducibly; it never copies object files or shared libraries.
Runtime flight startup does not execute this generator or create parameter files.

## Reproduce

The generated sources were validated with Python 3.10, the versions in `requirements.txt`, and:

- Repository acados commit `78cb9a72975ce2b5616c35936c9ac504a3ed7e5a`.
- Official acados Tera renderer `0.0.34`, Linux x86_64.
- Renderer SHA-256: `390063f34a8e13620564b4a136012270168e1421dd7920a747048749e1d99718`.

Start in the repository root. The acados libraries must already be built by the normal project build.
The following installs generator dependencies in `/tmp`; it does not modify system Python packages:

```bash
python3 -m pip install --target /tmp/agi_rate_mpc_python --no-deps \
  -r agilib/externals/acados_code_generator/requirements.txt
curl --fail --location \
  https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux \
  --output /tmp/agi_rate_mpc_t_renderer
chmod u+x /tmp/agi_rate_mpc_t_renderer
PYTHONPATH=/tmp/agi_rate_mpc_python \
TERA_PATH=/tmp/agi_rate_mpc_t_renderer \
  bash agilib/externals/acados_code_generator/acados_build_copy.sh
```

For the actual generation run, SciPy 1.8.0 was already available in the local Python installation;
CasADi 3.6.7, NumPy 1.24.4 and future-fstrings 1.2.0 were installed under `/tmp`.
The old acados checkout emits a warning because its declared supported CasADi versions end at 3.5.5.
The warning remains visible. With 3.6.7, generated C compilation, equation checks and solver hover checks passed;
this does not claim support for arbitrary acados/CasADi version combinations.

An existing virtual environment can instead be selected with `GENERATOR_PYTHON=/path/to/venv/bin/python`.
`ACADOS_SOURCE_DIR` and `TERA_PATH` may point to matching existing tools. For other architectures, use the matching renderer.
Rebuild all consumers after regeneration because generated dimensions and interfaces are compile-time constants.

To check symbolic dimensions and physical equations without generating or compiling C:

```bash
PYTHONPATH=/tmp/agi_rate_mpc_python:agilib/externals/acados-src/interfaces/acados_template \
  python3 agilib/externals/acados_code_generator/drone_model.py --check-model-only
```

The model checks cover hover force units, yaw-rate quaternion kinematics, and tilted-thrust acceleration.
The staging solver check verifies five successful hover solves and the full-horizon steady state.
Controller/reference/bounds/quaternion-regression tests belong to `agilib/tests/controller/controller_mpc_test.cpp`.
Software checks do not establish CM5 timing, flight-controller tracking bandwidth, or flight readiness.
