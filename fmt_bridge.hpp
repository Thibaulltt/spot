#ifndef SPOT__FMT_BRIDGE_HPP_
#define SPOT__FMT_BRIDGE_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 26/09/22
 * Path        : /fmt_bridge.hpp
 * Description : Makes the fmt library accessible by the project.
 *=============================================
 */

#include "glm_bridge.hpp"

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <fmt/chrono.h>
#include <fmt/std.h>

#include <experimental/string_view> // std::string_view before C++17

/// @brief Enables or disables debug messages in fmtdbg()
constexpr bool enable_debug = true;

/// @brief Temporary debug function.
template<typename...T>
void fmtdbg(fmt::format_string<T...> format, T&&... args){
	if (enable_debug) {
		auto full_str = fmt::format("[DEBUG] {}\n", format);
		fmt::print(full_str, args...);
	}
}

/// @brief This provides a formatting facility for the `glm::vec<>` types and its derivatives.
/// @tparam L_vector The length (number of components) of the vectors to format.
/// @tparam T_vector The data type of the vectors to format.
/// @tparam Q_vector The qualifier (precision & other info) of the vectors to format.
template <glm::length_t L_vector, typename T_vector, glm::qualifier Q_vector>
struct fmt::formatter<glm::vec<L_vector, T_vector, Q_vector>> {
	std::experimental::string_view format_specifier; ///< Holds the format specifier

	using vector_t = glm::vec<L_vector, T_vector, Q_vector>;

	/// @brief Parses the format specifier for the given vector.
	/// @details Fetches the numerical format specifier from the given format context, in order to use it when formatting the
	///   vector components.
	/// @param ctx The context to parse from.
	/// @returns A past-the-end iterator to the end of the formatting range
	constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
		auto iterator = ctx.begin(), end = ctx.end();
		// Special case : no formatting string
		if (iterator == nullptr) {
			format_specifier = std::experimental::string_view{nullptr, 0};
			return ctx.end();
		}
		// find end of format specifier range :
		while (*end != '}' && end != iterator) { end--; }
		// If no format specifier is given :
		if (iterator == end) {
			format_specifier = std::experimental::string_view{nullptr, 0};
		} else {
			// Fill fields for later use :
			format_specifier = {&*iterator, fmt::detail::to_unsigned(end - iterator)};
		}
		// Return the iterator at the end of the formatting range :
		return end;
	}

	/// @brief Formats the glm::vec<> type with the right format specifier.
	/// @tparam FormatContext The type of the formatting context to write to.
	/// @param vec The vector to print/format.
	/// @param ctx The context to write to.
	/// @returns A formatted string according to the previously parsed format specifier.
	template <typename FormatContext>
	auto format(const vector_t& vec, FormatContext& ctx) -> decltype(ctx.out()) {
		std::string vector_spec, component_spec;
		// If no format specifier, give default format string :
		component_spec = format_specifier.empty() ? "{}" : fmt::format("{{:{}}}", std::string(format_specifier));
		for (int i = 0; i < vec.length(); ++i) {
			vector_spec += component_spec + (i == vec.length() -1 ? "" : " ");
		}
		// Format the vector with the right amount of components :
		return format_to_length(ctx.out(), vector_spec, vec);
	}

	/// @brief Generic function for formatting a glm::vec<L, T, Q> to a string, given a formatting context and specification.
	/// @details This is not the function called when performing a call with a valid and known vector type. There are overloads present for
	///   the 2-vector, 3-vector and 4-vector types. Whenever this is the fallback, a formatting error is raised.
	/// @tparam L The length of the vector given to the function, in order to format it.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the vector being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the vector passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param vector_spec The format specification for the current vector.
	/// @param vec The vector to fetch data from.
	/// @throws A format error. If this is the default, the GLM type could not have been constructed.
	template <glm::length_t L, typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string vector_spec, const glm::vec<L, T, Q>& vec) {
		throw fmt::format_error(fmt::format("Error : attempting to format a {}-length vector from GLM. This should not be possible.", L));
	}

	/// @brief Overload of `format_to_length<L, FormatContextIterator, T, Q>()` for `L_local = 2`.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the vector being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the vector passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param vector_spec The format specification for the current vector.
	/// @param vec The vector to fetch data from.
	/// @returns The formatted vec2 representation.
	template <typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string vector_spec, const glm::vec<2, T, Q>& vec) {
		return fmt::format_to(iterator, vector_spec, vec.x, vec.y);
	}

	/// @brief Overload of `format_to_length<L, FormatContextIterator, T, Q>()` for `L_local = 3`.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the vector being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the vector passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param vector_spec The format specification for the current vector.
	/// @param vec The vector to fetch data from.
	/// @returns The formatted vec3 representation.
	template <typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string vector_spec, const glm::vec<3, T, Q>& vec) {
		return fmt::format_to(iterator, vector_spec, vec.x, vec.y, vec.z);
	}

	/// @brief Overload of `format_to_length<L, FormatContextIterator, T, Q>()` for `L_local = 4`.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the vector being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the vector passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param vector_spec The format specification for the current vector.
	/// @param vec The vector to fetch data from.
	/// @returns The formatted vec4 representation.
	template <typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string vector_spec, const glm::vec<4, T, Q>& vec) {
		return fmt::format_to(iterator, vector_spec, vec.x, vec.y, vec.z, vec.w);
	}
};

/// @brief This provides a formatting facility for the `glm::mat<>` types and its derivatives.
/// @tparam C_matrix The number of columns of the matrices to format.
/// @tparam R_matrix The number of rows of the matrices to format.
/// @tparam T_matrix The data type of the matrices to format.
/// @tparam Q_matrix The qualifier (precision & other info) of the matrices to format.
template <glm::length_t C_matrix, glm::length_t R_matrix, typename T_matrix, glm::qualifier Q_matrix>
struct fmt::formatter<glm::mat<C_matrix, R_matrix, T_matrix, Q_matrix>> {
	std::experimental::string_view format_specifier; ///< Holds the format specifier

	using matrix_t = glm::mat<C_matrix, R_matrix, T_matrix, Q_matrix>;

	/// @brief Parses the format specifier for the given matrix.
	/// @details Fetches the numerical format specifier from the given format context, in order to use it when formatting the
	///   matrix components.
	/// @param ctx The context to parse from.
	/// @returns A past-the-end iterator to the end of the formatting range
	constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
		auto iterator = ctx.begin(), end = ctx.end();
		// Special case : no formatting string
		if (iterator == nullptr) {
			format_specifier = std::experimental::string_view{nullptr, 0};
			return ctx.end();
		}
		// find end of format specifier range :
		while (*end != '}' && end != iterator) { end--; }
		// If no format specifier is given :
		if (iterator == end) {
			format_specifier = std::experimental::string_view{nullptr, 0};
		} else {
			// Fill fields for later use :
			format_specifier = {&*iterator, fmt::detail::to_unsigned(end - iterator)};
		}
		// Return the iterator at the end of the formatting range :
		return end;
	}

	/// @brief Formats the glm::vec<> type with the right format specifier.
	/// @tparam FormatContext The type of the formatting context to write to.
	/// @param vec The vector to print/format.
	/// @param ctx The context to write to.
	/// @returns A formatted string according to the previously parsed format specifier.
	template <typename FormatContext>
	auto format(const matrix_t& mat, FormatContext& ctx) -> decltype(ctx.out()) {
		std::string matrix_spec, component_spec;
		// If no format specifier, give default format string :
		component_spec = format_specifier.empty() ? "{}" : fmt::format("{{:{}}}", std::string(format_specifier));
		for (int i = 0; i < mat.length(); ++i) {
			matrix_spec += (i == 0 ? '[' : ' ') + component_spec + (i == mat.length() - 1 ? '\n' : ']');
		}
		// Format the vector with the right amount of components :
		return format_to_length(ctx.out(), matrix_spec, mat);
	}

	/// @brief Generic function for formatting a glm::mat<C, R, T, Q> to a string, given a formatting context and specification.
	/// @details This is not the function called when performing a call with a valid and known matrix type. There are overloads present for
	///   the 2xR-matrix, 3xR-matrix and 4xR-matrix types. Whenever this is the fallback, a formatting error is raised.
	/// @tparam C The number of columns of the matrix given to the function, in order to format it.
	/// @tparam R The number of rows of the matrix given to the function, in order to format it.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the matrix being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the matrix passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param matrix_spec The format specification for the current matrix.
	/// @param mat The matrix to fetch data from.
	/// @throws A format error. If this is the default, the GLM type could not have been constructed.
	template <glm::length_t C, glm::length_t R, typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string matrix_spec, const glm::mat<C, R, T, Q>& mat) {
		throw fmt::format_error(fmt::format("Error : attempting to format a {}-columned matrix from GLM. This should not be possible.", C));
	}

	/// @brief Overload of `format_to_length<C, R, FormatContextIterator, T, Q>()` for `C = 2`.
	/// @tparam R The number of rows of the matrix given to the function, in order to format it.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the matrix being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the matrix passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param matrix_spec The format specification for the current matrix.
	/// @param vec The matrix to fetch data from.
	/// @returns The formatted vec2 representation.
	template <glm::length_t R, typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string matrix_spec, const glm::mat<2, R, T, Q>& mat) {
		return fmt::format_to(iterator, matrix_spec, mat[0], mat[1]);
	}

	/// @brief Overload of `format_to_length<C, R, FormatContextIterator, T, Q>()` for `C = 3`.
	/// @tparam R The number of rows of the matrix given to the function, in order to format it.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the matrix being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the matrix passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param matrix_spec The format specification for the current matrix.
	/// @param vec The matrix to fetch data from.
	/// @returns The formatted vec3 representation.
	template <glm::length_t R, typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string matrix_spec, const glm::mat<3, R, T, Q>& mat) {
		return fmt::format_to(iterator, matrix_spec, mat[0], mat[1], mat[2]);
	}

	/// @brief Overload of `format_to_length<C, R, FormatContextIterator, T, Q>()` for `C = 4`.
	/// @tparam R The number of rows of the matrix given to the function, in order to format it.
	/// @tparam FormatContextIterator The type of the format context to write to.
	/// @tparam T The data type of the matrix being passed to the function.
	/// @tparam Q The qualifier (precision, and other info) of the matrix passed to the function.
	/// @param iterator The iterator giving the format to write to.
	/// @param matrix_spec The format specification for the current matrix.
	/// @param vec The matrix to fetch data from.
	/// @returns The formatted vec4 representation.
	template <glm::length_t R, typename FormatContextIterator, typename T, glm::qualifier Q>
	auto format_to_length(const FormatContextIterator& iterator, std::string matrix_spec, const glm::mat<4, R, T, Q>& mat) {
		return fmt::format_to(iterator, matrix_spec, mat[0], mat[1], mat[2], mat[3]);
	}
};

#endif //SPOT__FMT_BRIDGE_HPP_
