//
// Created by thib on 21/09/22.
// Checks the FIST method can output a known transform onto two of the same point clouds.
//

#include "../../spot_wrappers.hpp"
#include "../path_setup.hpp"

int main(int argc, char* argv[]) {
	constexpr glm::vec4::value_type factor = 15.0;
	glm::vec4 translation(uniform(engine) * factor, uniform(engine) * factor, uniform(engine) * factor, 0.0f);

	spot_wrappers::FISTWrapperSameModel known_transform(get_path_to_test_files("Datasets/models/bunny.off"), glm::identity<glm::mat4>(),translation);
	known_transform.compute_transformation(true);

	glm::mat4 extracted_transform = known_transform.get_computed_matrix();
	glm::vec4 extracted_translation = known_transform.get_computed_translation();

	constexpr float epsilon_precision = 1e-5f;
	bool matrix_close = glm::all(glm::bvec4(
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[0], extracted_transform[0], epsilon_precision)),
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[1], extracted_transform[1], epsilon_precision)),
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[2], extracted_transform[2], epsilon_precision)),
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[3], extracted_transform[3], epsilon_precision))
	));
	bool translate_close = glm::all(glm::epsilonEqual(translation, extracted_translation, epsilon_precision));

	fmt::print("Matrix extracted (should be close to id(4)) :\n");
	fmt::print("{: > 10.8f}\n", extracted_transform);
	fmt::print("Applied translation  : {: > 10.8f}\n", translation);
	fmt::print("Computed translation : {: > 10.8f}\n", extracted_translation);
	fmt::print("Difference           : {: > 10.8f}\n", glm::abs(translation - extracted_translation));
	fmt::print("matrix_close && translate_close --> {} && {}\n", matrix_close, translate_close);
	return (matrix_close && translate_close) ? EXIT_SUCCESS : EXIT_FAILURE;
}
