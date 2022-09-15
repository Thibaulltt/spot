#ifndef SPOT__PROGRAM_OPTIONS_HPP_
#define SPOT__PROGRAM_OPTIONS_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 15/09/22
 * Path        : /program_options.hpp
 * Description : Defines some options for the programs using SPOT.
 *=============================================
 */

#include <boost/program_options.hpp>
#include <fmt/core.h>

namespace program_options {

	/// @brief Prints a help message to the screen.
	constexpr void help_message();

	struct FIST_options {
	public:
		FIST_options(int argc, char* argv[]);
		~FIST_options() = default;

		/// @brief Prints a help message about the program.
		void help_message() const;

		/// @brief Prints the current status of the structure.
		void print_current_status();

		bool requested_help; ///< Did the user request help ?
		bool using_models; ///< Whether we are using models or randomly generated point clouds.
		bool using_reproducible_results; ///< Whether we want reproducibility or not in our results. Initializes a random seed with time() or a fixed value.
		std::string source_model_name; ///< If we're using models, the path to the file containing the source model.
		std::string target_model_name; ///< If we're using models, the path to the file containing the target model.
		std::uint32_t source_distribution_sample_count; ///< The number of points to generate in the source cloud. If using models, this is ignored.
		std::uint32_t target_distribution_sample_count; ///< The number of points to generate in the target cloud. If using models, this is ignored.

		std::uint32_t max_iteration_count; ///< The maximum number of iterations to perform.
		std::uint32_t max_direction_samples; ///< The maximum number of directions to sample for each iteration.
	};
}

#endif //SPOT__PROGRAM_OPTIONS_HPP_
