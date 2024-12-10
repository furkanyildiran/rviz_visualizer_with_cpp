cmake_minimum_required(VERSION 3.8)
project(arduinobot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)		#default
find_package(ament_cmake_python REQUIRED)  	#python
find_package(rclpy REQUIRED)			#python

ament_python_install_package(${PROJECT_NAME})	#python

install(PROGRAMS				#python
  ${PROJECT_NAME}/simple_serial_transmitter.py	#python
  ${PROJECT_NAME}/tf_publisher.py	#python
  DESTINATION lib/${PROJECT_NAME}		#python
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
