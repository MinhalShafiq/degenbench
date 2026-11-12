find_package(OpenMP REQUIRED)

add_library(degenbench_detectors src/features.cpp)
target_include_directories(degenbench_detectors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(degenbench_detectors PUBLIC degenbench_labeler)
target_compile_options(degenbench_detectors PRIVATE -Wall -Wextra -Wpedantic)

add_executable(detector_features tools/detector_features.cpp)
target_link_libraries(detector_features PRIVATE degenbench_detectors OpenMP::OpenMP_CXX)
target_compile_options(detector_features PRIVATE -Wall -Wextra -Wpedantic)

if(DEGENBENCH_BUILD_TESTS)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  add_executable(test_features test/test_features.cpp)
  target_link_libraries(test_features PRIVATE degenbench_detectors GTest::gtest_main)
  gtest_discover_tests(test_features DISCOVERY_TIMEOUT 60)
endif()
