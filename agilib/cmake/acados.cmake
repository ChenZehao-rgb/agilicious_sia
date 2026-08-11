# Reuse the vendored build when it is already present.  The previous logic ran
# an ExternalProject checkout of an unfixed `master` on every configure, which
# made otherwise-local SITL and target builds depend on network availability.
set(ACADOS_ROOT "${PROJECT_SOURCE_DIR}/externals/acados-src" CACHE PATH
    "Path to an installed or vendored acados tree")
option(FETCH_ACADOS "Download and build acados when ACADOS_ROOT is incomplete" ON)

set(ACADOS_LIB_DIR "${ACADOS_ROOT}/lib")
set(ACADOS_READY TRUE)
foreach(ACADOS_REQUIRED_FILE
    "${ACADOS_LIB_DIR}/libacados.so"
    "${ACADOS_LIB_DIR}/libhpipm.so"
    "${ACADOS_LIB_DIR}/libblasfeo.so"
    "${ACADOS_ROOT}/include/acados_c/ocp_nlp_interface.h")
  if(NOT EXISTS "${ACADOS_REQUIRED_FILE}")
    set(ACADOS_READY FALSE)
  endif()
endforeach()

# Acados installs target-specific shared objects in its source tree.  In
# particular, an x86_64 build left in the vendored directory is otherwise easy
# to pick up while cross-compiling for a Raspberry Pi.  Read e_machine directly
# from the ELF header so this check does not depend on host `file`/`readelf`
# output or locale.
function(_agilib_validate_acados_elf_arch)
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ACADOS_TARGET_PROCESSOR)

  if(ACADOS_TARGET_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(ACADOS_EXPECTED_MACHINE "003e")
    set(ACADOS_EXPECTED_ARCH "x86-64")
  elseif(ACADOS_TARGET_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(ACADOS_EXPECTED_MACHINE "00b7")
    set(ACADOS_EXPECTED_ARCH "AArch64")
  elseif(ACADOS_TARGET_PROCESSOR MATCHES "^(arm|armv[5-8].*)$")
    set(ACADOS_EXPECTED_MACHINE "0028")
    set(ACADOS_EXPECTED_ARCH "ARM")
  elseif(ACADOS_TARGET_PROCESSOR MATCHES "^(x86|i[3-6]86)$")
    set(ACADOS_EXPECTED_MACHINE "0003")
    set(ACADOS_EXPECTED_ARCH "x86")
  else()
    message(WARNING
      "Cannot validate Acados ELF architecture for target processor "
      "'${CMAKE_SYSTEM_PROCESSOR}'.")
    return()
  endif()

  foreach(ACADOS_ELF_LIBRARY
      "${ACADOS_LIB_DIR}/libacados.so"
      "${ACADOS_LIB_DIR}/libhpipm.so"
      "${ACADOS_LIB_DIR}/libblasfeo.so")
    if(NOT EXISTS "${ACADOS_ELF_LIBRARY}")
      message(FATAL_ERROR
        "Required Acados library is missing after setup: ${ACADOS_ELF_LIBRARY}")
    endif()

    file(READ "${ACADOS_ELF_LIBRARY}" ACADOS_ELF_MAGIC OFFSET 0 LIMIT 4 HEX)
    string(TOLOWER "${ACADOS_ELF_MAGIC}" ACADOS_ELF_MAGIC)
    if(NOT ACADOS_ELF_MAGIC STREQUAL "7f454c46")
      message(FATAL_ERROR
        "Acados library is not an ELF binary: ${ACADOS_ELF_LIBRARY}")
    endif()

    # EI_DATA (offset 5) describes byte order; e_machine is the two-byte field
    # at offset 18.  Normalize e_machine to big-endian hexadecimal for matching.
    file(READ "${ACADOS_ELF_LIBRARY}" ACADOS_ELF_DATA OFFSET 5 LIMIT 1 HEX)
    file(READ "${ACADOS_ELF_LIBRARY}" ACADOS_ELF_MACHINE OFFSET 18 LIMIT 2 HEX)
    string(TOLOWER "${ACADOS_ELF_DATA}" ACADOS_ELF_DATA)
    string(TOLOWER "${ACADOS_ELF_MACHINE}" ACADOS_ELF_MACHINE)
    if(ACADOS_ELF_DATA STREQUAL "01")
      string(SUBSTRING "${ACADOS_ELF_MACHINE}" 0 2 ACADOS_MACHINE_LOW_BYTE)
      string(SUBSTRING "${ACADOS_ELF_MACHINE}" 2 2 ACADOS_MACHINE_HIGH_BYTE)
      set(ACADOS_ELF_MACHINE
          "${ACADOS_MACHINE_HIGH_BYTE}${ACADOS_MACHINE_LOW_BYTE}")
    elseif(NOT ACADOS_ELF_DATA STREQUAL "02")
      message(FATAL_ERROR
        "Acados library has an unsupported ELF byte order: ${ACADOS_ELF_LIBRARY}")
    endif()

    if(NOT "${ACADOS_ELF_MACHINE}" STREQUAL "${ACADOS_EXPECTED_MACHINE}")
      message(FATAL_ERROR
        "Acados library architecture mismatch for ${ACADOS_ELF_LIBRARY}: "
        "ELF e_machine=0x${ACADOS_ELF_MACHINE}, but target processor "
        "'${CMAKE_SYSTEM_PROCESSOR}' expects ${ACADOS_EXPECTED_ARCH} "
        "(0x${ACADOS_EXPECTED_MACHINE}). Set ACADOS_ROOT to libraries built "
        "for the target architecture, or rebuild Acados with the target toolchain.")
    endif()
  endforeach()

  message(STATUS
    "Validated Acados ELF libraries for ${ACADOS_EXPECTED_ARCH} "
    "(${CMAKE_SYSTEM_PROCESSOR})")
endfunction()

if(ACADOS_READY)
  _agilib_validate_acados_elf_arch()
  message(STATUS "Using existing Acados at ${ACADOS_ROOT}")
elseif(FETCH_ACADOS)
  message(STATUS "Getting Acados...")

  configure_file(
    cmake/acados_download.cmake
    ${PROJECT_SOURCE_DIR}/externals/acados-download/CMakeLists.txt)

  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/acados-download
    OUTPUT_QUIET
    ERROR_QUIET)
  if(result)
    message(FATAL_ERROR "Download of Acados failed: ${result}")
    message(STATUS "${PROJECT_SOURCE_DIR}/externals/acados-download")
  endif()

  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/acados-download/)
  if(result)
    message(FATAL_ERROR "Build step for Acados failed: ${result}")
  endif()

  message(STATUS "Acados downloaded and built!")
  _agilib_validate_acados_elf_arch()
else()
  message(FATAL_ERROR
    "Acados was not found under ${ACADOS_ROOT}. Set ACADOS_ROOT or enable FETCH_ACADOS.")
endif()

add_library(hpipm_lib SHARED IMPORTED)
set_target_properties(hpipm_lib PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${ACADOS_ROOT}/external/hpipm/include
  IMPORTED_LOCATION ${ACADOS_LIB_DIR}/libhpipm.so)

add_library(blasfeo_lib SHARED IMPORTED)
set_target_properties(blasfeo_lib PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${ACADOS_ROOT}/external/blasfeo/include
  IMPORTED_LOCATION ${ACADOS_LIB_DIR}/libblasfeo.so)

add_library(acados_lib SHARED IMPORTED)
set_target_properties(acados_lib PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${ACADOS_ROOT}/include
  IMPORTED_LOCATION ${ACADOS_LIB_DIR}/libacados.so)

set(acados_INCLUDE_DIR "${ACADOS_ROOT}/include"
  "${ACADOS_ROOT}/external/blasfeo/include"
  "${ACADOS_ROOT}/external/hpipm/include")

set(acados_LIB_DIR "${ACADOS_LIB_DIR}")


add_library(acados_agi INTERFACE IMPORTED GLOBAL)
target_link_libraries(acados_agi INTERFACE hpipm_lib blasfeo_lib acados_lib)
