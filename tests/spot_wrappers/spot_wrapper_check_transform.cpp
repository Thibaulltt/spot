//
// Created by thib on 26/09/22.
//

#include "../../src/spot_wrappers.hpp"
#include "../../external/fmt_bridge.hpp"
#include "../path_setup.hpp"

using float_t = glm::vec3::value_type;

/// @brief Checks the ``FISTWrapperSameModel`` actually performs a translation of _all_ vertices given a known translation.
/// @param factor The scale factor to apply to the random translation applied. If none are given, 1.0 is assumed.
/// @param epsilon_acceptable The epsilon to check the vertex coordinate differences with. Defaults to the fp32 ULP (1e-7).
/// @note See the code for a troubling behaviour for GLM types.
/// @returns True if all vertices are translated correctly up to an epsilon value.
short fist_wrapper_check_translation_only(const float_t factor = 1.0, const float_t epsilon_acceptable = 1e-7) {
	/** Note about this test :
	 *
	 * This is supposed to be simple. Load a model, apply some translation/transformation to it and check to see if the result
	 * is about what you'd expect, up to an epsilon-value of difference for all vertices of the meshes.
	 *
	 * HOWEVER.
	 *
	 * GLM types (or the Point<> type, who knows) does not seem to behave as expected. A regular fp32 float should be precise
	 * up to around 1e-7 (that's about their ULP). However, for some reason the following test detects nearly _half_ of all
	 * coordinates of the bunny to be different to 1e-5 (!!!) than the expected value.
	 *
	 * This is troubling, especially since we're trying to gauge the precision of registration algorithms. The delta between
	 * two such algorithms can be contained in a 1e-5 delta (or less !).
	 * I do not know how to proceed here.
	 */
	glm::vec3 total_difference{}; // The difference computed for all vertices of the distributions
	glm::vec3 translation(uniform(engine) * factor, uniform(engine) * factor, uniform(engine) * factor);
	spot_wrappers::FISTWrapperSameModel model(get_path_to_test_files("Datasets/models/bunny.off"), glm::identity<glm::mat4>(), glm::vec4{translation, 0.0f});

	auto source = model.get_source_distribution();
	auto target = model.get_target_distribution();
	using index_t = decltype(source)::size_type; // aka vector<>::size_type
	std::vector<bool> different_coordinates(source.size());

	fmtdbg("Starting difference : {}", total_difference);
	for (index_t i = 0; i < source.size(); ++i) {
		// Gather computed value and perform the expected computation here to check difference :
		glm::vec3 real = glm::to_vec(target[i]);
		glm::vec3 expected = glm::to_vec(source[i]) + translation;
		// Compute difference and check which components are different :
		glm::vec3 local_difference = glm::abs(expected - real);
		glm::bvec3 different_components = glm::epsilonEqual(local_difference, glm::vec3{0.0f}, epsilon_acceptable);
		// Add it to the total sum.
		total_difference += local_difference;
		different_coordinates[i] = not glm::all(different_components);
		// Deubg print : shows the delta and the binary comparison to a null vector :
		//fmtdbg("Difference : [{: > 10.8f}] ==> [{:b}]", local_difference, different_components);
	}
	total_difference /= static_cast<float>(source.size());

	std::uint32_t different_coordinates_count = 0;
	for (auto different_coordinates_value : different_coordinates) {
		different_coordinates_count += static_cast<std::uint32_t>(different_coordinates_value);
	}
	fmt::print("There were {: >5d}/{: >5d} deviating from the original value of translation + original coordinates.\n", different_coordinates_count, source.size());

	fmt::print("Expected translation : [{: > 10.8f}]\n", translation);
	fmt::print("Difference           : [{: > 10.8f}]\n", total_difference);

	return glm::all(glm::epsilonEqual(total_difference, glm::vec3(0.0f), epsilon_acceptable)) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/// @brief Checks the ``FISTWrapperSameModel`` actually performs the right transform on all vertices of the model.
/// @param angle The rotation angle (in DEGREES) to spin the model around a random axis. If none are given, 30.0 is assumed.
/// @param epsilon_acceptable The epsilon to check the vertex coordinate differences with. Defaults to the fp32 ULP (1e-7).
/// @note See the code for a troubling behaviour for GLM types.
/// @returns True if all vertices are translated correctly up to an epsilon value.
short fist_wrapper_check_transform_only(const float_t angle = 30.0f, const float_t epsilon_acceptable = 1e-7) {
	// Generate a random axis for rotation, to rotate the model :
	glm::vec3 random_axis = glm::normalize(glm::sphericalRand(1.0f));
	glm::quat random_quat = glm::angleAxis(glm::radians(angle), random_axis);
	// Generate a 3x3 matrix from this quaternion :
	glm::mat3 transform = glm::mat3_cast(random_quat);
	glm::vec3 translation(0.0f);
	spot_wrappers::FISTWrapperSameModel model(get_path_to_test_files("Datasets/models/bunny.off"), glm::mat4(transform), glm::vec4{translation, 0.0f});

	auto source = model.get_source_distribution();
	auto target = model.get_target_distribution();
	using index_t = decltype(source)::size_type; // aka vector<>::size_type
	std::vector<bool> different_coordinates(source.size());

	glm::vec3 total_difference{}; // The difference computed for all vertices of the distributions
	for (index_t i = 0; i < source.size(); ++i) {
		// Gather computed value and perform the expected computation here to check difference :
		glm::vec3 real = glm::to_vec(target[i]);
		glm::vec3 expected = glm::to_vec(source[i]) * transform;
		// Compute difference and check which components are different :
		glm::vec3 local_difference = glm::abs(expected - real);
		glm::bvec3 different_components = glm::epsilonEqual(local_difference, glm::vec3{0.0f}, epsilon_acceptable);
		// Add it to the total sum.
		total_difference += local_difference;
		different_coordinates[i] = not glm::all(different_components);
		// Deubg print : shows the delta and the binary comparison to a null vector :
		//fmtdbg("Difference : [{: > 10.8f}] ==> [{:b}]", local_difference, different_components);
	}
	total_difference /= static_cast<float>(source.size());

	std::uint32_t different_coordinates_count = 0;
	for (auto different_coordinates_value : different_coordinates) {
		different_coordinates_count += static_cast<std::uint32_t>(different_coordinates_value);
	}
	fmt::print("There were {: >5d}/{: >5d} deviating from the original value of (original coordinates) * transform.\n", different_coordinates_count, source.size());

	fmt::print("Difference           : [{: > 10.8f}]\n", total_difference);

	return glm::all(glm::epsilonEqual(total_difference, glm::vec3(0.0f), epsilon_acceptable)) ? EXIT_SUCCESS : EXIT_FAILURE;
}

/// @brief Converts an exit code to a string. Can only parse EXIT_{SUCCESS|FAILURE}.
const char* exit_code_to_string(short exit_code) {
	switch (exit_code) {
		case EXIT_SUCCESS: return "SUCCESS";
		case EXIT_FAILURE: return "FAILURE";
		default: return "<unknown>";
	}
}

int main(int argc, char* argv[]) {
	// Some values for the tests :
	constexpr float_t angle_rotation = 30.0f;
	constexpr float_t translation_factor = 15.0f;
	constexpr float_t maximum_epsilon_tolerable = 2e-5f;

	auto is_translation_nearly_equal = fist_wrapper_check_translation_only(translation_factor, maximum_epsilon_tolerable);
	auto is_transform_nearly_equal = fist_wrapper_check_transform_only(angle_rotation, maximum_epsilon_tolerable);

	fmt::print("\nFinal results :\n");
	fmt::print("\t- Result of translation_only : {}\n", exit_code_to_string(is_translation_nearly_equal));
	fmt::print("\t- Result of transform_only : {}\n", exit_code_to_string(is_transform_nearly_equal));

	return ((is_translation_nearly_equal && is_transform_nearly_equal) == EXIT_SUCCESS) ? EXIT_SUCCESS : EXIT_FAILURE;
}
