include_directories (${TARGET_INCLUDE_DIRS})

link_libraries (
    jetnet_static
    ${TARGET_LIBS}
)

add_executable (jetnet_build_darknet_model build_darknet_model.cpp)
add_executable (jetnet_run_yolo run_yolo.cpp)
add_executable (jetnet_validate_yolo validate_yolo.cpp)

install (TARGETS
    jetnet_build_darknet_model
    jetnet_run_yolo
    jetnet_validate_yolo
    DESTINATION "bin"
)
