ENABLE_TESTING()

ADD_EXECUTABLE(fist_translation_only
	fist_translation_only.cpp
	../../UnbalancedSliced.cpp
	../../micro_benchmark.cpp
)
TARGET_LINK_LIBRARIES(fist_translation_only
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt::fmt
	PUBLIC assimp::assimp
	PUBLIC glm_bridge
)
ADD_TEST(
	NAME test_fist_translation_only
	COMMAND fist_translation_only
)
