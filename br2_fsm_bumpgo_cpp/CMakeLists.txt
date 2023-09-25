cmake_minimum_required(VERSION 3.5)
project(br2_fsm_bumpgo_cpp)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
)

include_directories(include)

add_executable(bumpgo
  src/br2_fsm_bumpgo_cpp/BumpGoNode.cpp
  src/bumpgo_main.cpp
)

add_executable(bumpgo_ex1
  src/br2_fsm_bumpgo_cpp/BumpGoNodeEx1.cpp
  src/bumpgo_main_ex1.cpp
)

add_executable(bumpgo_ex2_ol
  src/br2_fsm_bumpgo_cpp/BumpGoNodeEx2OL.cpp
  src/bumpgo_main_ex2_ol.cpp
)

add_executable(bumpgo_ex2_cl
  src/br2_fsm_bumpgo_cpp/BumpGoNodeEx2CL.cpp
  src/bumpgo_main_ex2_cl.cpp
)

ament_target_dependencies(bumpgo ${dependencies})
ament_target_dependencies(bumpgo_ex1 ${dependencies})
ament_target_dependencies(bumpgo_ex2_ol ${dependencies})
ament_target_dependencies(bumpgo_ex2_cl ${dependencies})


install(TARGETS
  bumpgo
  bumpgo_ex1
  bumpgo_ex2_ol
  bumpgo_ex2_cl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
