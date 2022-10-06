ENABLE_TESTING()

ADD_EXECUTABLE(model_range_based_operations model_range_based_operations.cpp)
TARGET_LINK_LIBRARIES(model_range_based_operations
	PUBLIC fmt_bridge
	PUBLIC glm_bridge
)
ADD_TEST(
	NAME test_model_range_based_operations
	COMMAND model_range_based_operations
)

ADD_EXECUTABLE(model_precision_checks model_precision_checks.cpp)
TARGET_LINK_LIBRARIES(model_precision_checks
	PUBLIC fmt_bridge
	PUBLIC glm_bridge
)
ADD_TEST(
	NAME test_model_precision_checks
	COMMAND model_precision_checks
)