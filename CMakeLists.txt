cmake_minimum_required(VERSION 3.26...4.0)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics_python_interface_coppeliasim_zmq)

find_package(Eigen3 REQUIRED)
if(UNIX AND NOT APPLE)
    find_package(ZeroMQ CONFIG REQUIRED)
else ()
    find_package(cppzmq REQUIRED)
endif ()

add_subdirectory(submodules/dqrobotics/python/pybind11)

pybind11_add_module(_core
    src/module.cpp
    src/interfaces/coppeliasim/DQ_CoppeliaSimInterface_py.cpp
    src/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ_py.cpp
)

set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(submodules/dqrobotics/python/cpp)
include_directories(submodules/dqrobotics/cpp-interface-coppeliasim/include) # used by cpp-interface-coppeliasim-zmq
add_subdirectory(submodules/dqrobotics/cpp-interface-coppeliasim-zmq)

target_include_directories(_core PUBLIC
    submodules/dqrobotics/cpp-interface-coppeliasim/include
    submodules/dqrobotics/cpp-interface-coppeliasim-zmq/include
)

if(APPLE)
    target_link_libraries(_core PRIVATE 
    dqrobotics 
    dqrobotics-interface-coppeliasim-zmq
    cppzmq
    Eigen3::Eigen
    )
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(_core PRIVATE 
    dqrobotics 
    dqrobotics-interface-coppeliasim-zmq 
    zmq
    Eigen3::Eigen
    )
endif()
