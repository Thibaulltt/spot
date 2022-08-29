//
// Created by thib on 26/08/22.
//

#include "../../model.hpp"
#include <glm/gtx/io.hpp>

constexpr float max_fp32_tolerance = 1e-6;

constexpr bool float_equality(float _lhs, float _rhs) {
	return std::fabs(_lhs - _rhs) < max_fp32_tolerance;
}

int main(int argc, char* argv[]) {
	glm::vec3 translation = glm::sphericalRand(1.0f);

	std::vector<glm::vec3> positions{
		glm::sphericalRand(1.0f),
		glm::sphericalRand(1.0f),
		glm::sphericalRand(1.0f)
	};
	std::vector<glm::uvec3> triangles;

	Model m_original{positions, triangles};
	m_original.apply_translation(translation);

	bool all_valid = true;

	for (int i = 0; i < positions.size(); ++i) {
		positions[i] += translation;
		for (int j = 0; j < 3; ++j) {
			if (not float_equality(positions[i][j], m_original.positions[i][j])) {
				all_valid = false;
			}
		}
	}
	return all_valid ? EXIT_SUCCESS : EXIT_FAILURE;
}
