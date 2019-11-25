cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

project(xythetac)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_IDRS})


#add_executable(xythetac src/xythetaMHA.cpp src/environment_navxythetaC.cpp src/embedded_multi_heuristics.cpp) 
add_executable(xythetac src/xythetaARA.cpp src/environment_navxythetaC.cpp src/dubins.cpp)


target_link_libraries(xythetac ${SBPL_LIBRARIES})
