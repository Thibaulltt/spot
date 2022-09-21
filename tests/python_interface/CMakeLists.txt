ENABLE_TESTING()

ADD_EXECUTABLE(fist_wrapper_check_existence
	fist_wrapper_check_existence.cpp
)
TARGET_LINK_LIBRARIES(fist_wrapper_check_existence
	PUBLIC spot_wrappers
)
ADD_TEST(
	NAME test_fist_wrapper_check_existence
	COMMAND fist_wrapper_check_existence
)
