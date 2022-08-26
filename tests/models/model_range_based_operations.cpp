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
	glm::vec3 translation = glm::random(glm::vec3{});

	std::vector<glm::vec3> positions{
		glm::random(translation),
		glm::random(translation),
		glm::random(translation)
	};
	std::vector<glm::uvec3> triangles;

	Model m_original{positions, triangles};
	m_original.apply_translation(translation);

	for (int i = 0; i < positions.size(); ++i) {
		positions[i] += translation;
		for (int j = 0; j < 3; ++j) {
			if (not float_equality(positions[i][j], m_original.positions[i][j])) {
				return EXIT_FAILURE;
			}
		}
	}
	return EXIT_SUCCESS;
}
