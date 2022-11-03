ENABLE_TESTING()

ADD_EXECUTABLE(fist_translation_only
	fist_translation_only.cpp
	../../src/UnbalancedSliced.cpp
	../../src/micro_benchmark.cpp
)
TARGET_LINK_LIBRARIES(fist_translation_only
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt_bridge
	PUBLIC glm_bridge
)
ADD_TEST(
	NAME test_fist_translation_only
	COMMAND fist_translation_only
)

ADD_EXECUTABLE(fist_rigidbody
	fist_rigidbody.cpp
	../../src/UnbalancedSliced.cpp
	../../src/micro_benchmark.cpp
)
TARGET_LINK_LIBRARIES(fist_rigidbody
	PUBLIC OpenMP::OpenMP_CXX
	PUBLIC fmt_bridge
	PUBLIC glm_bridge
)
ADD_TEST(
	NAME test_fist_rigidbody
	COMMAND fist_rigidbody
)