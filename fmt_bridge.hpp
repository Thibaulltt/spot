#ifndef SPOT__FMT_BRIDGE_HPP_
#define SPOT__FMT_BRIDGE_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 26/09/22
 * Path        : /fmt_bridge.hpp
 * Description : Makes the fmt library accessible by the project.
 *=============================================
 */

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <fmt/chrono.h>
#include <fmt/std.h>

#include "glm_bridge.hpp"

/// @brief Enables or disables debug messages in fmtdbg()
constexpr bool enable_debug = true;

//region --- Free functions : debug utilities, converters, and environment setup ---
/// @brief Temporary debug function.
template<typename...T>
void fmtdbg(fmt::format_string<T...> format, T&&... args){
	if (enable_debug) {
		auto full_str = fmt::format("[DEBUG] {}\n", format);
		fmt::print(full_str, args...);
	}
}

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

template <> struct fmt::formatter<glm::mat4> {
	/// @brief Parses format specifications. No-op in the case of glm::bvec3.
	constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
		return ctx.end();
	}

	/// @brief Formats the vector given the length of it, and the internal data type.
	template <typename FormatContext>
	auto format(const glm::mat4& m, FormatContext& ctx) const -> decltype(ctx.out()) {
		return fmt::format_to(ctx.out(),
			"[[{: >10.6f} {: >10.6f} {: >10.6f} {: >10.6f}],\n"
			" [{: >10.6f} {: >10.6f} {: >10.6f} {: >10.6f}]\n"
			" [{: >10.6f} {: >10.6f} {: >10.6f} {: >10.6f}]\n"
			" [{: >10.6f} {: >10.6f} {: >10.6f} {: >10.6f}]]",
			m[0][0], m[0][1], m[0][2], m[0][3],
			m[1][0], m[1][1], m[1][2], m[1][3],
			m[2][0], m[2][1], m[2][2], m[2][3],
			m[3][0], m[3][1], m[3][2], m[3][3]);
	}
};

template <> struct fmt::formatter<glm::vec3> {
	/// @brief Parses format specifications. No-op in the case of glm::vec3.
	constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
		return ctx.end();
	}

	/// @brief Formats the vector given the length of it, and the internal data type.
	template <typename FormatContext>
	auto format(const glm::vec3& v, FormatContext& ctx) const -> decltype(ctx.out()) {
		return fmt::format_to(ctx.out(), "[{} {} {}]", v[0], v[1], v[2]);
	}
};

template <> struct fmt::formatter<glm::vec4> {
	/// @brief Parses format specifications. No-op in the case of glm::vec4.
	constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
		return ctx.end();
	}

	/// @brief Formats the vector given the length of it, and the internal data type.
	template <typename FormatContext>
	auto format(const glm::vec4& v, FormatContext& ctx) const -> decltype(ctx.out()) {
		return fmt::format_to(ctx.out(), "[{} {} {} {}]", v[0], v[1], v[2], v[3]);
	}
};

#endif //SPOT__FMT_BRIDGE_HPP_
