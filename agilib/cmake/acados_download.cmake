cmake_minimum_required(VERSION 3.0.0)

project(acados-project)

include(ExternalProject)
ExternalProject_Add(ACADOS_PROJECT
  GIT_REPOSITORY    https://github.com/uzh-rpg/acados.git
  # Version used by the generated MPC wrapper checked into this repository.
  GIT_TAG           78cb9a72975ce2b5616c35936c9ac504a3ed7e5a
  SOURCE_DIR        "@ACADOS_ROOT@"
  INSTALL_DIR       "@ACADOS_ROOT@"
  CMAKE_ARGS        "-DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}"
                    "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}"
                    "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
                    "-DCMAKE_INSTALL_PREFIX=@ACADOS_ROOT@"
)
