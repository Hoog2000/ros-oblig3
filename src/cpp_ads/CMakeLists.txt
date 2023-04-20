cmake_minimum_required(VERSION 3.8)
project(cpp_ads)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crane_interface REQUIRED)
#find_package(ADS REQUIRED)
#find_package(ads_library_pkg REQUIRED)


#set(THREADS_PREFER_FLAG ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#include_directories(${ADS_INCLUDE_DIRS})
#link_directories(${ADS_LIBRARE_DIRS})

find_library(
    LIBADS_LIBRARY
    NAMES AdsLib
    HINTS "../../ADS/build"
)

add_executable(ads_node src/ads.cpp)


target_compile_features(ads_node PUBLIC c_std_99 cxx_std_17) #require c99 and cpp 17

target_include_directories(ads_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(ads_node PRIVATE "../../ADS/AdsLib")


target_link_libraries(ads_node PUBLIC ${LIBADS_LIBRARY} Threads::Threads)

ament_target_dependencies(ads_node PUBLIC rclcpp crane_interface)

install(TARGETS ads_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
