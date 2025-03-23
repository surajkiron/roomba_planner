cmake_minimum_required(VERSION 3.0.2)
project(interview-planning)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find Python and its development libraries
find_package(PythonLibs REQUIRED)
find_package(PythonInterp REQUIRED)

# Find NumPy
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Include directories
include_directories(${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

# Add matplotlibcpp.h
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add executable
add_executable(CheckPath src/main.cpp)

# Link libraries
target_link_libraries(CheckPath PRIVATE Eigen3::Eigen ${PYTHON_LIBRARIES})

# Add matplotlibcpp.h to the target
target_include_directories(CheckPath PRIVATE include ${PYTHON_INCLUDE_DIRS} ${NUMPY_INCLUDE_DIR})