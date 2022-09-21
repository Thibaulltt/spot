ENABLE_TESTING()

ADD_EXECUTABLE(model_range_based_operations model_range_based_operations.cpp)
TARGET_LINK_LIBRARIES(model_range_based_operations
	PUBLIC fmt::fmt
	PUBLIC glm_bridge
)
ADD_TEST(
	NAME test_model_range_based_operations
	COMMAND model_range_based_operations
)
