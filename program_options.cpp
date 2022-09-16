//
// Created by thib on 15/09/22.
//

#include "program_options.hpp"

namespace program_options {

	FIST_options::FIST_options(int argc, char **argv) {
		namespace bpo = boost::program_options;

		bpo::options_description options("Program options for FIST");
		options.add_options()
			("help,h", bpo::value<bool>(&this->requested_help), "Prints this help message")
			("reproducible,r", bpo::value<bool>(&this->using_reproducible_results)->default_value(true), "Enable reproducible results (fixed random seed) or not")
			("source,s", bpo::value<std::string>(&this->source_model_name)->default_value(""), "The source model file (OFF only) for this run of FIST.")
			("target,t", bpo::value<std::string>(&this->target_model_name)->default_value(""), "The target model file (OFF only) for this run of FIST.")
			("source_samples", bpo::value<std::uint32_t>(&this->source_distribution_sample_count)->default_value( 5000), "The number of samples to generate in the source distribution")
			("target_samples", bpo::value<std::uint32_t>(&this->target_distribution_sample_count)->default_value(10000), "The number of samples to generate in the target distribution")
			("iterations,i", bpo::value<std::uint32_t>(&this->max_iteration_count)->default_value(20), "The maximum number of iterations to perform")
			("directions,d", bpo::value<std::uint32_t>(&this->max_direction_samples)->default_value(100), "The maximum number of directions to sample for each iteration")
		;

		// Parse the arguments :
		bpo::variables_map vmap;
		bpo::store(bpo::parse_command_line(argc, argv, options), vmap);
		bpo::notify(vmap);

		const bool has_either_model_names = not this->source_model_name.empty() or not this->target_model_name.empty();
		const bool has_both_model_names = not this->source_model_name.empty() and not this->target_model_name.empty();

		if (this->requested_help) {
			this->help_message();
			this->using_models = false;
		}

		if (has_either_model_names and not has_both_model_names) {
			fmt::print("Error : only one model name was passed to the program.\n");
			fmt::print("The program will use randomly generated point clouds instead.\n");
			this->using_models = false;
		} else if (not has_either_model_names) {
			this->using_models = false;
		} else {
			this->using_models = true;
		}
	}

	constexpr void FIST_options::help_message() {
		fmt::print("<Help message unavailable for now>\n");
	}

}
