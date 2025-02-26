#ifndef SPOT__GLM_BRIDGE_HPP_
#define SPOT__GLM_BRIDGE_HPP_
#pragma once

/*=============================================
 * Creator     : thib
 * Created on  : 26/08/22
 * Path        : /glm_bridge.hpp
 * Description : Provides a slew of GLM includes to use the lib elsewhere.
 *=============================================
 */

#include <glm/glm.hpp>					/* General-purpose header for GLM : includes all GLSL symbols.     */
#include <glm/gtx/io.hpp>				/* Provides operator<< and operator>> functions for `std::fstream` */
#include <glm/gtc/epsilon.hpp>			/* Provides glm::epsilon[Not]Equal(), equality with epsilon checks */
#include <glm/gtc/random.hpp>			/* Randoms : linear, circular, spherical, disk, ball, and Gaussian */
#include <glm/ext/scalar_constants.hpp>	/* Provides some scalar constants such as PI. */
#include <glm/ext/matrix_transform.hpp>	/* Contains mat<>::identity ! */

namespace glm {

	/// @brief Template overload to make epsilonEqual accept glm::mat3 comparisons.
	GLM_CONSTEXPR GLM_FUNC_QUALIFIER bool epsilonEqual(glm::mat3 const& a, glm::mat3 const& b, glm::mat3 const& epsilon) {
		glm::bvec3 row0 = lessThan(abs(a[0] - b[0]), epsilon[0]);
		glm::bvec3 row1 = lessThan(abs(a[1] - b[1]), epsilon[1]);
		glm::bvec3 row2 = lessThan(abs(a[2] - b[2]), epsilon[2]);
		return glm::all(glm::bvec3{glm::all(row0), glm::all(row1), glm::all(row2)});
	}

}

#endif //SPOT__GLM_BRIDGE_HPP_
