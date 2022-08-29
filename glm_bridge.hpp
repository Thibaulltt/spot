#ifndef SPOT__GLM_BRIDGE_HPP_
#define SPOT__GLM_BRIDGE_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 26/08/22
 * Path        : /glm_bridge.hpp
 * Description : Provides a slew of GLM includes to use the lib elsewhere.
 *=============================================
 */

#include <glm/glm.hpp>         /* General-purpose header for GLM : includes all GLSL symbols.     */
#include <glm/gtx/io.hpp>      /* Provides operator<< and operator>> functions for `std::fstream` */
#include <glm/gtc/epsilon.hpp> /* Provides glm::epsilon[Not]Equal(), equality with epsilon checks */
#include <glm/gtc/random.hpp>  /* Randoms : linear, circular, spherical, disk, ball, and Gaussian */
#include <glm/ext/scalar_constants.hpp>

namespace glm {

	/// @brief Template overload to make epsilonEqual accept glm::mat3 comparisons.
	template <>
	bool epsilonEqual(glm::mat3 const& a, glm::mat3 const& b, glm::mat3 const& epsilon) {
		glm::bvec3 row0 = lessThan(abs(a[0] - b[0]), epsilon[0]);
		glm::bvec3 row1 = lessThan(abs(a[1] - b[1]), epsilon[1]);
		glm::bvec3 row2 = lessThan(abs(a[2] - b[2]), epsilon[2]);
		return glm::all(glm::bvec3{glm::all(row0), glm::all(row1), glm::all(row2)});
	}

}

#include <fmt/format.h>

/// @brief Template overload for fmt::formatter using the glm::bvec3 type.
template <> struct fmt::formatter<glm::bvec3> {
	/// @brief Parses format specifications. No-op in the case of glm::bvec3.
	constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
		return ctx.end();
	}

	/// @brief Formats the vector given the length of it, and the internal data type.
	template <typename FormatContext>
	auto format(const glm::bvec3& p, FormatContext& ctx) const -> decltype(ctx.out()) {
		return fmt::format_to(ctx.out(), "({}, {}, {})", p.x, p.y, p.z);
	}
};


#endif //SPOT__GLM_BRIDGE_HPP_
