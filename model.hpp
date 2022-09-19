#ifndef SPOT__MODEL_HPP_
#define SPOT__MODEL_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 24/08/22
 * Path        : /model_importer.hpp
 * Description : Utility functions to load a 3D model from a file.
 *=============================================
 */

#include "glm_bridge.hpp"
#include "Point.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <string>
#include <vector>

/// @brief Loads a given path in order to extract a single model from it.
const aiScene* load_model_file(const std::string load_path);

/// @brief Prints the contents of a loaded `aiScene` pointer.
/// @note Only there for debugging reasons.
void print_aiScene_contents(const aiScene* scene);

/// @brief Simple model class, holds positions and triangles of an OFF file.
struct Model {
	Model();
	Model(const std::vector<glm::vec3>& vertices, std::vector<glm::uvec3> triangles);
	Model(const Model& _other);
	~Model() = default;

	/// @brief Applies a matrix transform to the positions.
	void apply_transform(glm::mat3);
	/// @brief Applies a translation to all positions.
	void apply_translation(glm::vec3);

	std::vector<Point<3, float>> positions;
	std::vector<glm::uvec3> triangles;
};

/// @brief Load a given OFF file and returns its contents already converted to a std::vector<Point>.
/// @returns A model with the file contents. If the file could not be loaded, returns an empty model.
Model load_off_file(const std::string& path);

#include "model.impl.hpp"

#endif //SPOT__MODEL_HPP_
