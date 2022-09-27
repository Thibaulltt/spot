//
// Created by thib on 26/09/22.
//

#include "../../spot_wrappers.hpp"
#include "../../fmt_bridge.hpp"
#include "../path_setup.hpp"

int main(int argc, char* argv[]) {
	constexpr glm::vec4::value_type factor = 15.0;
	constexpr glm::vec4::value_type epsilon_acceptable = 1e-5;

	glm::vec3 total_difference{}; // The difference computed for all vertices of the distributions
	glm::vec4 translation(uniform(engine) * factor, uniform(engine) * factor, uniform(engine) * factor, 0.0f);
	spot_wrappers::FISTWrapperSameModel model(get_path_to_test_files("Datasets/models/bunny.off"), glm::identity<glm::mat4>(), translation);

	auto source = model.get_source_distribution();
	auto target = model.get_target_distribution();
	using index_t = decltype(source)::size_type; // aka vector<>::size_type

	fmtdbg("Starting difference : {}", total_difference);
	for (index_t i = 0; i < source.size();) {
		glm::vec3 temp_difference = glm::vec3{};
		index_t j = i;
		for (; j < i+1000 && j < source.size(); ++j) {
			auto local_difference = target[j] - source[j];
//			fmtdbg("Difference : [{: > 10.8f}] ==> [{:b}]",
//				glm::to_vec(local_difference),
//				glm::epsilonEqual(glm::vec4{glm::to_vec(local_difference), 0.0}, translation, epsilon_acceptable)
//			);
			temp_difference += glm::to_vec(local_difference);
		}
		fmtdbg("For {: >4d} vectors, mean translation was {: > 10.8f}", j-i, temp_difference / static_cast<float>(j-i));
		i = j;
		temp_difference /= static_cast<float>(source.size());
		total_difference += temp_difference;
	}
	fmtdbg("Total un-divided difference : [{: > 10.8f}]\n", total_difference);

	fmt::print("Expected translation : [{: > 10.8f}]\n", translation);
	fmt::print("Computed translation : [{: > 10.8f}]\n", total_difference);
	fmt::print("Difference           : [{: > 10.8f}]\n", translation - glm::vec4{total_difference, 0.0f});

	return glm::all(glm::epsilonEqual(glm::vec4(total_difference, 0.0f), translation, epsilon_acceptable)) ? EXIT_SUCCESS : EXIT_FAILURE;
}
