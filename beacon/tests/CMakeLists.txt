
ament_add_gtest(vff_test vff_test.cc)
ament_target_dependencies(vff_test ${dependencies})
target_link_libraries(vff_test ${PROJECT_NAME})
