cmake_minimum_required(VERSION 3.4...3.18)
project(modern_robotics)
find_package(Eigen3 3.0 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED )
add_subdirectory(pybind11)

pybind11_add_module(modern_robotics src/main.cpp src/modern_robotics.cpp src/modern_robotics_relative.cpp src/MR_Indy7.cpp )

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(modern_robotics
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen jsoncpp_lib)
