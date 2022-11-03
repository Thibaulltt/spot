#ifndef SPOT__MODEL_HPP_
#define SPOT__MODEL_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 24/08/22
 * Path        : /model_importer.hpp
 * Description : Utility functions to load a 3D model from a file.
 *=============================================
 */

#include "../external/glm_bridge.hpp"
#include "Point.h"

#include <string>
#include <vector>

/// @brief Simple model class, holds positions and triangles of an OFF file.
struct Model {
	Model();
	Model(const std::vector<glm::vec3>& vertices, std::vector<glm::uvec3> triangles);
	Model(const Model& _other);
	Model(Model&& _other) noexcept;
	~Model() = default;

	/// @brief Applies a matrix transform to the positions.
	/// @param transform The transform to apply. Should be rigid body, but can be anything.
	void apply_transform(glm::mat3 transform);
	/// @brief Applies a translation to all positions.
	/// @param translation The translation to apply.
	void apply_translation(glm::vec3 translation);
	/// @brief Applies a scaling factor to the positions of the model.
	/// @param scaling The scaling factor to apply.
	/// @param center_before_scaling If true, this centers the mean of the model to the origin, scales and replaces the
	///   model to its original position.
	void apply_scaling(double scaling, bool center_before_scaling);

	std::vector<Point<3, float>> positions;
	std::vector<glm::uvec3> triangles;
};

/// @brief Load a given OFF file and returns its contents already converted to a std::vector<Point>.
/// @returns A model with the file contents. If the file could not be loaded, returns an empty model.
Model load_off_file(const std::string& path);

#include "model.impl.hpp"

#endif //SPOT__MODEL_HPP_
