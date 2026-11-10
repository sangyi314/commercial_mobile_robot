cmake_minimum_required(VERSION 3.16)
project(cmr_tracking LANGUAGES CXX)

include(CTest)
option(CMR_TRACKING_WITH_ROS "Export the pure library through ament" OFF)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(cmr_tracking src/time_contract.cpp src/covariance.cpp)
target_compile_features(cmr_tracking PUBLIC cxx_std_17)
set_target_properties(cmr_tracking PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(cmr_tracking PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(cmr_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cmr_tracking PUBLIC Eigen3::Eigen)
install(TARGETS cmr_tracking EXPORT export_cmr_tracking
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  add_executable(tracking_contract_test test/contract_test.cpp)
  target_link_libraries(tracking_contract_test PRIVATE cmr_tracking GTest::gtest_main)
  foreach(case TimeBoundary CovarianceAvailability FrameTwist RandomTimestamp)
    add_test(NAME P201.${case} COMMAND tracking_contract_test --gtest_filter=P201.${case})
  endforeach()
endif()

if(CMR_TRACKING_WITH_ROS)
  find_package(ament_cmake REQUIRED)
  ament_export_targets(export_cmr_tracking HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)
  ament_package()
endif()
