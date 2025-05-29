cmake_minimum_required(VERSION 3.26...4.0)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics_python_interface_coppeliasim_zmq)

if(APPLE)
    find_package(cppzmq)
endif()

add_subdirectory(submodules/dqrobotics/python/pybind11)

pybind11_add_module(_core
    src/module.cpp
    #interfaces/copppeliasim-zmq
    src/interfaces/coppeliasim/DQ_CoppeliaSimInterface_py.cpp
    src/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ_py.cpp
)

target_include_directories(_core PUBLIC
        submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim/include/
        submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq/include/
        submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq/submodules/zmqRemoteApi/clients/cpp/
        submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq/submodules/jsoncons/include/
)

set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(cpp)
add_subdirectory(submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq)

if(APPLE)
    target_link_libraries(_core PRIVATE dqrobotics dqrobotics-interface-coppeliasim-zmq cppzmq)
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(_core PRIVATE dqrobotics dqrobotics-interface-coppeliasim-zmq zmq)
endif()
