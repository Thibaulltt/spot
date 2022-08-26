ENABLE_TESTING()

ADD_EXECUTABLE(model_loader_inspection model_file_inspection.cpp)
TARGET_LINK_LIBRARIES(model_loader_inspection
	PUBLIC fmt::fmt
	PUBLIC assimp::assimp
)
ADD_TEST(
	NAME test_model_loader_inspection
	COMMAND model_loader_inspection
)

ADD_EXECUTABLE(model_range_based_operations model_range_based_operations.cpp)
TARGET_LINK_LIBRARIES(model_range_based_operations
	PUBLIC fmt::fmt
	PUBLIC assimp::assimp
	PUBLIC glm_bridge
)
ADD_TEST(
	NAME test_model_range_based_operations
	COMMAND model_range_based_operations
)
