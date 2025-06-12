cmake_minimum_required(VERSION 3.15...3.26)

project(marinholab_working_needlemanipulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Werror=return-type -fPIC)

if(APPLE)
    include_directories(
            /usr/local/include/
            /usr/local/include/eigen3
            /opt/homebrew/include
            /opt/homebrew/include/eigen3
    )
endif()

if(WIN32)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
    include
    /usr/include/eigen3/
    submodules/dqrobotics/cpp/include/
    )

add_subdirectory(submodules/dqrobotics/python/pybind11)
pybind11_add_module(_core MODULE
        src/core.cpp
        src/M3_SerialManipulatorSimulatorFriendly.cpp
    )

# CMAKE was not passing BUILD_SHARED_LIBS nicely to the add_subdirectory.
# https://thatonegamedev.com/cpp/cmake/how-to-manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(submodules/dqrobotics/python/cpp)

target_link_libraries(_core PRIVATE dqrobotics)

target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})