cmake_minimum_required(VERSION 3.26...4.0)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics_python_interface_coppeliasim_zmq)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# COPPELIASIM INTERFACE
add_definitions(-DSIM_REMOTEAPICLIENT_OBJECTS)

if(APPLE)
    find_package(cppzmq)
    message("Building not supported on this platform.")
    include_directories(
        /usr/local/include/
        /usr/local/include/eigen3
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
    )
    add_compile_options(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(UNIX AND NOT APPLE)
    add_compile_options(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual -fPIC)
endif()

if(WIN32)
    message("Building not supported on this platform.")
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

add_subdirectory(submodules/dqrobotics/python/pybind11)

include_directories(
    /usr/include/eigen3/

    submodules/dqrobotics/python/cpp/include/
    submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim/include/
    submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq/include/
    submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq/submodules/zmqRemoteApi/clients/cpp/
    submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq/submodules/jsoncons/include/
)

pybind11_add_module(_core

    src/module.cpp
    #interfaces/copppeliasim-zmq
    src/interfaces/coppeliasim/DQ_CoppeliaSimInterface_py.cpp
    src/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ_py.cpp
)

# CMAKE was not passing BUILD_SHARED_LIBS nicely to the add_subdirectory.
# https://thatonegamedev.com/cpp/cmake/how-to-manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(submodules/dqrobotics/python/cpp)
add_subdirectory(submodules/dqrobotics/python/interfaces/cpp-interface-coppeliasim-zmq)

if(APPLE)
    target_link_libraries(_core PRIVATE dqrobotics dqrobotics-interface-coppeliasim-zmq cppzmq)
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(_core PRIVATE dqrobotics dqrobotics-interface-coppeliasim-zmq zmq)
endif()
