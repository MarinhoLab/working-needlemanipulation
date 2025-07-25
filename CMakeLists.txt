# https://pybind11.readthedocs.io/en/stable/compiling.html
cmake_minimum_required(VERSION 3.15...3.29)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(marinholab_working_needlemanipulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

set(PYBIND11_FINDPYTHON ON)
add_subdirectory(submodules/dqrobotics/python/pybind11)
pybind11_add_module(_core MODULE
        src/core.cpp
        src/M3_SerialManipulatorSimulatorFriendly.cpp
    )

target_include_directories(_core PRIVATE
    include
    submodules/dqrobotics/cpp/include/
    #submodules/dqrobotics/cpp-interface-qpoases/include
    #submodules/qpOASES/include
    )

# CMAKE was not passing BUILD_SHARED_LIBS nicely to the add_subdirectory.
# https://thatonegamedev.com/cpp/cmake/how-to-manage-dependencies-with-cmake/
set(BUILD_SHARED_LIBS FALSE CACHE BOOL "x" FORCE)
add_subdirectory(submodules/dqrobotics/python/cpp)
#add_subdirectory(submodules/qpOASES)

target_link_libraries(_core PRIVATE Eigen3::Eigen dqrobotics) #qpOASES
# Version is dynamically obtained. See pyproject.toml
# Activating this causes issues with Windows compilation.
# target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})