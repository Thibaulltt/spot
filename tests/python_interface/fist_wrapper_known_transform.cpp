//
// Created by thib on 21/09/22.
// Checks the FIST method can output a known transform onto two of the same point clouds.
//

#include "../../spot_wrappers.hpp"

int main(int argc, char* argv[]) {
	glm::vec4 translation(engine(), engine(), engine(), 0.0f);

	spot_wrappers::FISTWrapperSameModel known_transform("/home/thib/Documents/data/medmax/meshes/Test/bunny.off", glm::identity<glm::mat4>(),translation);
	known_transform.compute_transformation(true);

	glm::mat4 extracted_transform = known_transform.get_transform_matrix();
	glm::vec4 extracted_translation = known_transform.get_transform_translation();

	constexpr float epsilon_precision = 1e-5f;
	bool matrix_close = glm::all(glm::bvec4(
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[0], extracted_transform[0], epsilon_precision)),
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[1], extracted_transform[1], epsilon_precision)),
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[2], extracted_transform[2], epsilon_precision)),
		glm::all(glm::epsilonEqual(glm::identity<glm::mat4>()[3], extracted_transform[3], epsilon_precision))
	));
	bool translate_close = glm::all(glm::epsilonEqual(extracted_translation, extracted_translation, epsilon_precision));

	fmt::print("Matrix extracted (should be close to id(4)) :\n");
	fmt::print("{}\n", extracted_transform);
	fmt::print("Applied translation :\n");
	fmt::print("{}\n", translation);
	fmt::print("Computed translation :\n");
	fmt::print("{}\n", extracted_translation);
	return (matrix_close && translate_close) ? EXIT_SUCCESS : EXIT_FAILURE;
}
