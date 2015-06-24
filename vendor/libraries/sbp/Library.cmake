file(GLOB DUNE_SBP_FILES user/vendor/libraries/sbp/*.c)

set_source_files_properties(${DUNE_SBP_FILES}
    PROPERTIES COMPILE_FLAGS "${DUNE_C_FLAGS} ${DUNE_C_FLAGS_STRICT} -std=c99")

list(APPEND DUNE_VENDOR_FILES ${DUNE_SBP_FILES})

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries)
