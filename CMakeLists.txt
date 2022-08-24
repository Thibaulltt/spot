CMAKE_MINIMUM_REQUIRED(VERSION 3.15)
PROJECT(spot
	VERSION 1
	DESCRIPTION "SPOT : Sliced Partial Optimal Transport. Method applied to ICP and histogram matching."
	LANGUAGES C CXX
)

# Only use C++11, and export compile commands :
SET(CMAKE_CXX_STANDARD 11)

MESSAGE(STATUS "This can only compile on macOS or Linux.")
MESSAGE(STATUS "Use the Visual Studio pre-packaged solutions otherwise.")

FIND_PACKAGE(OpenMP REQUIRED)
FIND_PACKAGE(fmt REQUIRED)

# Compile the executables with the same compile flags as the Makefile.
# Only difference : no optimization flags (determined by CMAKE_BUILD_TYPE).
IF ( ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
	SET(SPOT_COMPILE_FLAGS -march=native  -I. -L/usr/local/opt/libomp/lib -I/usr/local/opt/libomp/include -Xpreprocessor -fopenmp -lomp -fno-signed-zeros -fno-trapping-math  -openmp  -funroll-loops -mavx)
ELSE()
	SET(SPOT_COMPILE_FLAGS -fopenmp -mavx -I.)
ENDIF()

# Enable the compilation flags :
ADD_COMPILE_OPTIONS(${SPOT_COMPILE_FLAGS})

# Add the executables :
ADD_EXECUTABLE(FIST
	mainFIST.cpp
	UnbalancedSliced.cpp
	micro_benchmark.cpp
)
TARGET_LINK_LIBRARIES(FIST
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt::fmt
)
ADD_EXECUTABLE(colorTransfer
	mainColorTransfer.cpp
	UnbalancedSliced.cpp
	micro_benchmark.cpp
)
TARGET_LINK_LIBRARIES(colorTransfer
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt::fmt
)
