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
	using index_t = decltype(source)::size_type;

	fmtdbg("Starting difference : {}", total_difference);
	for (index_t i = 0; i < source.size(); ++i) {
		auto local_difference = source[i] - target[i];
		total_difference += glm::abs(glm::to_vec(local_difference));
	}
	fmtdbg("Total un-divided difference : {}\n", total_difference);
	total_difference /= static_cast<float>(source.size());

	fmt::print("Expected translation : {}", translation);
	fmt::print("Computed translation : {}", total_difference);

	return glm::all(glm::epsilonEqual(glm::vec4(total_difference, 0.0f), translation, epsilon_acceptable)) ? EXIT_SUCCESS : EXIT_FAILURE;
}
