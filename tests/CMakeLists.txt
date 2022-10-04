# This performs some tests on the classes designed in the project root.

ENABLE_TESTING()

SET(SPOT_BASE_TEST_DIR "${CMAKE_SOURCE_DIR}/")
CONFIGURE_FILE(path_setup.hpp.in path_setup.hpp @ONLY)

# Macro used to produce single-file executables using the wrapper types using pybind11 :
MACRO(add_python_interface_test_single_file test_name)
	ADD_EXECUTABLE(${test_name}
		${test_name}.cpp
		)
	TARGET_LINK_LIBRARIES(${test_name}
		PUBLIC spot_wrappers
		PUBLIC pybind11::headers
		PUBLIC pybind11::embed
		PUBLIC pybind11::pybind11
		)
	SET_TARGET_PROPERTIES(${test_name} PROPERTIES
		INTERPROCEDURAL_OPTIMIZATION ON
		CXX_VISIBILITY_PRESET "hidden"
		VISIBILITY_INLINES_HIDDEN ON
		)
	ADD_TEST(
		NAME test_${test_name}
		COMMAND ${test_name}
	)
ENDMACRO()

ADD_SUBDIRECTORY(spot_wrappers)    # Tests the SPOT wrappers (C++ side).
ADD_SUBDIRECTORY(models)           # Tests the model class and its functions.
ADD_SUBDIRECTORY(fist_method)      # Tests the FIST method itself, using the 'raw' classes.
#ADD_SUBDIRECTORY(python_interface) # Tests the Python interface to the SPOT wrappers.
