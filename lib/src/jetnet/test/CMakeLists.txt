find_package (GTest REQUIRED COMPONENTS
    gtest
    gmock
)

include_directories (
    ${TARGET_INCLUDE_DIRS}
    gtest
    gmock
)

# unit test executables
add_executable (test_cv_letterbox_pre_processor data/test_cv_letterbox_pre_processor.cpp)

target_link_libraries (test_cv_letterbox_pre_processor
    jetnet_static
    ${TARGET_LIBS}
    gtest
    gmock
    pthread
)

# add test to ctest so you can run all test executables with a single build target
add_test (NAME run_test_cv_letterbox_pre_processor COMMAND test_cv_letterbox_pre_processor)
