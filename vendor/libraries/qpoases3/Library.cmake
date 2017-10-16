file(GLOB DUNE_QPOASES3_FILES user/vendor/libraries/qpoases3/*.c)

set_source_files_properties(${DUNE_QPOASES3_FILES}
    PROPERTIES COMPILE_FLAGS "${DUNE_C_FLAGS}")

list(APPEND DUNE_VENDOR_FILES ${DUNE_QPOASES3_FILES})

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries)
