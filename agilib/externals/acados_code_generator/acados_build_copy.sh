#!/bin/bash

FILE_NAME="drone_model"

AGILICIOUS_ROOT=$(git rev-parse --show-toplevel)

ACADOS_ROOT=${AGILICIOUS_ROOT}/agilib/externals/acados-src

pip3 install -e ${ACADOS_ROOT}/interfaces/acados_template

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"${ACADOS_ROOT}/lib"
export ACADOS_SOURCE_DIR="${ACADOS_ROOT}"

rm -rf c_generated_code/
rm -rf ${AGILICIOUS_ROOT}/agilib/src/controller/mpc/acados/*.c
rm -rf ${AGILICIOUS_ROOT}/agilib/src/controller/mpc/acados/*.o
rm -rf ${AGILICIOUS_ROOT}/agilib/include/agilib/controller/mpc/acados/*.h


python3 ${FILE_NAME}.py

cp c_generated_code/acados_solver_${FILE_NAME}.c ${AGILICIOUS_ROOT}/agilib/src/controller/mpc/acados/
cp -r c_generated_code/${FILE_NAME}_model/ ${AGILICIOUS_ROOT}/agilib/src/controller/mpc/acados/
cp -r c_generated_code/${FILE_NAME}_cost/ ${AGILICIOUS_ROOT}/agilib/src/controller/mpc/acados/
cp c_generated_code/acados_solver_${FILE_NAME}.h ${AGILICIOUS_ROOT}/agilib/include/agilib/controller/mpc/acados/


