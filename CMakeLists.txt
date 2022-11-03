CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
PROJECT(spot
	VERSION 1
	DESCRIPTION "SPOT : Sliced Partial Optimal Transport. Method applied to ICP and histogram matching."
	LANGUAGES C CXX
)

INCLUDE(GNUInstallDirs) # Needed for default CMAKE_INSTALL_*DIR variables

# Only use C++11, and export compile commands :
SET(CMAKE_CXX_STANDARD 14)

MESSAGE(STATUS "This can only compile on macOS or Linux.")
MESSAGE(STATUS "Use the Visual Studio pre-packaged solutions otherwise.")

# Find required packages :
FIND_PACKAGE(OpenMP REQUIRED)
FIND_PACKAGE(fmt REQUIRED)
FIND_PACKAGE(glm REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS program_options)
FIND_PACKAGE(pybind11 REQUIRED)

# Compile the executables with the same compile flags as the Makefile.
# Only difference : no optimization flags (determined by CMAKE_BUILD_TYPE).
IF ( ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
	SET(SPOT_COMPILE_FLAGS -march=native  -I. -L/usr/local/opt/libomp/lib -I/usr/local/opt/libomp/include -Xpreprocessor -fopenmp -lomp -fno-signed-zeros -fno-trapping-math  -openmp  -funroll-loops -mavx)
ELSE()
	SET(SPOT_COMPILE_FLAGS -fopenmp -mavx -I.)
ENDIF()

# Enable the compilation flags :
ADD_COMPILE_OPTIONS(${SPOT_COMPILE_FLAGS})

# Add the definitions to glm :
ADD_LIBRARY(glm_bridge INTERFACE external/glm_bridge.hpp)
TARGET_LINK_LIBRARIES(glm_bridge INTERFACE glm)
# Add the definitions to fmtlib :
ADD_LIBRARY(fmt_bridge INTERFACE external/fmt_bridge.hpp)
TARGET_LINK_LIBRARIES(fmt_bridge INTERFACE fmt::fmt)

MESSAGE(STATUS "Current CMake version executed is ${CMAKE_VERSION}")
IF(DEFINED CMAKE_ENABLE_TESTS)
	INCLUDE(CTest)
	MESSAGE(STATUS "Enabled testing.")
	ENABLE_TESTING()
	ADD_SUBDIRECTORY(tests)
ENDIF()

# Add the executables :
ADD_EXECUTABLE(FIST
	src/mainFIST.cpp
	src/UnbalancedSliced.cpp
	src/micro_benchmark.cpp
	src/program_options.cpp
)
TARGET_LINK_LIBRARIES(FIST
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt_bridge
	PUBLIC glm_bridge
	PUBLIC Boost::program_options
)
ADD_EXECUTABLE(colorTransfer
	src/mainColorTransfer.cpp
	src/UnbalancedSliced.cpp
	src/micro_benchmark.cpp
)
TARGET_LINK_LIBRARIES(colorTransfer
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt_bridge
)

ADD_LIBRARY(spot_wrappers SHARED
	src/spot_wrappers.hpp
	src/spot_wrappers.cpp
	src/micro_benchmark.cpp
	src/UnbalancedSliced.cpp
)
TARGET_LINK_LIBRARIES(spot_wrappers
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt_bridge
	PUBLIC glm_bridge
	PUBLIC pybind11::pybind11
)

pybind11_add_module(_spot src/spot_wrappers_python_interface.cpp)
TARGET_LINK_LIBRARIES(_spot PRIVATE spot_wrappers)

INSTALL(
	TARGETS _spot
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/spot
)
