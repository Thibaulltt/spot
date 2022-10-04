//
// Created by thib on 26/09/22.
//

#include "../../spot_wrappers.hpp"
#include "../../fmt_bridge.hpp"
#include "../path_setup.hpp"

using float_t = glm::vec3::value_type;

/// @brief Checks the ``FISTWrapperSameModel`` actually performs a translation of _all_ vertices given a known translation.
/// @param factor The scale factor to apply to the random translation applied. If none are given, 1.0 is assumed.
/// @param epsilon_acceptable The epsilon to check the vertex coordinate differences with. Defaults to the fp32 ULP (1e-7).
/// @note See the code for a troubling behaviour for GLM types.
/// @returns True if all vertices are translated correctly up to an epsilon value.
bool fist_wrapper_check_translation_only(const float_t factor = 1.0, const float_t epsilon_acceptable = 1e-7) {
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
	// glm::vec3 translation(uniform(engine) * factor, uniform(engine) * factor, uniform(engine) * factor);
	glm::vec3 translation(5.0f, 5.0f, 5.0f);
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
		different_coordinates[i] = glm::all(different_components);
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

int main(int argc, char* argv[]) {
	constexpr float_t maximum_epsilon_tolerable = 1e-5f;
	auto is_translation_nearly_equal = fist_wrapper_check_translation_only(15.0f, maximum_epsilon_tolerable);
	return is_translation_nearly_equal ? EXIT_SUCCESS : EXIT_FAILURE;
}
