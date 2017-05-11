file(GLOB DUNE_DUBIN_FILES user/vendor/libraries/aw-dubins-curves/*.c)

set_source_files_properties(${DUNE_DUBIN_FILES}
    PROPERTIES COMPILE_FLAGS "${DUNE_C_FLAGS}")

list(APPEND DUNE_VENDOR_FILES ${DUNE_DUBIN_FILES})

set(DUNE_VENDOR_INCS_DIR ${DUNE_VENDOR_INCS_DIR}
  ${PROJECT_SOURCE_DIR}/user/vendor/libraries)
