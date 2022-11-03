#ifndef SPOT__PROGRAM_OPTIONS_HPP_
#define SPOT__PROGRAM_OPTIONS_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 15/09/22
 * Path        : /program_options.hpp
 * Description : Defines some options for the programs using SPOT.
 *=============================================
 */

#include "../external/fmt_bridge.hpp"

#include <boost/program_options.hpp>

namespace program_options {

	/// @brief This class implements a baseline argument parser.
	/// @details It only defines two flags for help ("help" and "h"). The other programs must subclass this and add
	/// their own arguments.
	class base_program_options {
	public:
		base_program_options(int argc, char* argv[]);
		~base_program_options() = default;

		/// @brief Checks if the given options are passed to the program by the user (single arguments).
		bool has_argument(std::string single_option) const;
		/// @brief Checks if the given argument is passed to the program by the user (short and long arguments).
		bool has_argument(std::string long_option, std::string short_option) const;

	protected:
		/// @brief Simple typedef to ``boost::program_options::options_description``.
		using options_description = boost::program_options::options_description;
		/// @brief Simple typedef to ``boost::program_options::variable_map``.
		using variable_map = boost::program_options::variables_map;

	protected:
		std::unique_ptr<options_description> options; ///< The options' descriptions and (optionally) default values.
		std::unique_ptr<variable_map> cmdline_map;    ///< The map containing arguments passed to the program.
	};

	struct FIST_options {

		/// @brief Prints a help message about the program.
		static void help_message();

	public:
		FIST_options(int argc, char* argv[]);
		~FIST_options() = default;

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
