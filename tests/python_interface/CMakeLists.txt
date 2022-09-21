ENABLE_TESTING()

ADD_EXECUTABLE(fist_wrapper_check_existence
	fist_wrapper_check_existence.cpp
)
TARGET_LINK_LIBRARIES(fist_wrapper_check_existence
	PUBLIC spot_wrappers
	PUBLIC pybind11::headers
	PUBLIC pybind11::embed
	PUBLIC pybind11::pybind11
)
SET_TARGET_PROPERTIES(fist_wrapper_check_existence PROPERTIES
	INTERPROCEDURAL_OPTIMIZATION ON
	CXX_VISIBILITY_PRESET "hidden"
	VISIBILITY_INLINES_HIDDEN ON
)
ADD_TEST(
	NAME test_fist_wrapper_check_existence
	COMMAND fist_wrapper_check_existence
)

ADD_EXECUTABLE(fist_wrapper_known_transform
	fist_wrapper_known_transform.cpp
	)
TARGET_LINK_LIBRARIES(fist_wrapper_known_transform
	PUBLIC spot_wrappers
	PUBLIC pybind11::headers
	PUBLIC pybind11::embed
	PUBLIC pybind11::pybind11
)
SET_TARGET_PROPERTIES(fist_wrapper_known_transform PROPERTIES
	INTERPROCEDURAL_OPTIMIZATION ON
	CXX_VISIBILITY_PRESET "hidden"
	VISIBILITY_INLINES_HIDDEN ON
)
ADD_TEST(
	NAME test_fist_wrapper_known_transform
	COMMAND fist_wrapper_known_transform
)
