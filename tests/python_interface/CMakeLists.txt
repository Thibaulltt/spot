ENABLE_TESTING()

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

add_python_interface_test_single_file(fist_wrapper_check_existence)
add_python_interface_test_single_file(fist_wrapper_check_transform)
add_python_interface_test_single_file(fist_wrapper_known_transform)
add_python_interface_test_single_file(fist_wrapper_random_distributions)

#ADD_EXECUTABLE(fist_wrapper_check_existence
#	fist_wrapper_check_existence.cpp
#)
#TARGET_LINK_LIBRARIES(fist_wrapper_check_existence
#	PUBLIC spot_wrappers
#	PUBLIC pybind11::headers
#	PUBLIC pybind11::embed
#	PUBLIC pybind11::pybind11
#)
#SET_TARGET_PROPERTIES(fist_wrapper_check_existence PROPERTIES
#	INTERPROCEDURAL_OPTIMIZATION ON
#	CXX_VISIBILITY_PRESET "hidden"
#	VISIBILITY_INLINES_HIDDEN ON
#)
#ADD_TEST(
#	NAME test_fist_wrapper_check_existence
#	COMMAND fist_wrapper_check_existence
#)
#
#ADD_EXECUTABLE(fist_wrapper_known_transform
#	fist_wrapper_known_transform.cpp
#)
#TARGET_LINK_LIBRARIES(fist_wrapper_known_transform
#	PUBLIC spot_wrappers
#	PUBLIC pybind11::headers
#	PUBLIC pybind11::embed
#	PUBLIC pybind11::pybind11
#)
#SET_TARGET_PROPERTIES(fist_wrapper_known_transform PROPERTIES
#	INTERPROCEDURAL_OPTIMIZATION ON
#	CXX_VISIBILITY_PRESET "hidden"
#	VISIBILITY_INLINES_HIDDEN ON
#)
#ADD_TEST(
#	NAME test_fist_wrapper_known_transform
#	COMMAND fist_wrapper_known_transform
#)
#
#ADD_EXECUTABLE(fist_wrapper_random_distributions
#	fist_wrapper_random_distributions.cpp
#)
#TARGET_LINK_LIBRARIES(fist_wrapper_random_distributions
#	PUBLIC spot_wrappers
#	PUBLIC pybind11::headers
#	PUBLIC pybind11::embed
#	PUBLIC pybind11::pybind11
#)
#SET_TARGET_PROPERTIES(fist_wrapper_random_distributions PROPERTIES
#	INTERPROCEDURAL_OPTIMIZATION ON
#	CXX_VISIBILITY_PRESET "hidden"
#	VISIBILITY_INLINES_HIDDEN ON
#)
#ADD_TEST(
#	NAME test_fist_wrapper_random_distributions
#	COMMAND fist_wrapper_random_distributions
#)
