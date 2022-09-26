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


#endif //SPOT__FMT_BRIDGE_HPP_
