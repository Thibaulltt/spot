//
// Created by thibault on 21/09/22.
//

#include "spot_wrappers.hpp"

PYBIND11_MODULE(_spot, spot_module) {
	// Those argument literals are __really__ useful ...
	using namespace pybind11::literals;
	// Shorten the definition of documentation string literals :
	using pydoc = pybind11::doc;

	// Typedefs to the types to wrap :
	using Stats = micro_benchmarks::TimeSeriesStatistics;
	using Timings = micro_benchmarks::TimingsLogger;
	using FISTBase = spot_wrappers::FIST_BaseWrapper;
	using FISTRandom = spot_wrappers::FISTWrapperRandomModels;
	using FISTSame = spot_wrappers::FISTWrapperSameModel;
	using FISTDifferent = spot_wrappers::FISTWrapperDifferentModels;

	// Typedefs and helper lambdas for time-related structs/functions :
	using coarse_duration_t = micro_benchmarks::coarse_duration_t;
	using timelength_t = micro_benchmarks::coarse_duration_t::rep;
	auto duration_to_time = [](const micro_benchmarks::duration_t& duration) -> timelength_t { return std::chrono::duration_cast<coarse_duration_t>(duration).count(); };
	auto coarse_to_time = [](const micro_benchmarks::coarse_duration_t& coarse_duration) -> timelength_t { return coarse_duration.count(); };

	/* ----------------------------------------- */
	/* Some general definitions for the module : */
	/* ----------------------------------------- */
	spot_module.doc() = "A set of wrappers around the SPOT method : from Sliced Partial Optimal Transport by Bonneel and Coeurjolly (2019)";
	spot_module.def("enable_reproducible_runs", [](){ spot_wrappers::set_enable_reproducible_runs(true); })
		.doc() = "Enables reproducible runs : sets the random engine to be initialized with a constant value instead of a timestamp.";
	spot_module.def("disable_reproducible_runs", [](){ spot_wrappers::set_enable_reproducible_runs(false); })
		.doc() = "Disables reproducible runs : sets the random engine to be initialized with a timestamp instead of a constant value.";

	/* ------------------------ */
	/* Declare used GLM types : */
	/* ------------------------ */
	define_glm_type_matrix(glm::mat4{}, "mat4", spot_module);
	define_glm_type_matrix(glm::mat3{}, "mat3", spot_module);
	define_glm_type_vector(glm::vec4{}, "vec4", spot_module);
	define_glm_type_vector(glm::vec3{}, "vec3", spot_module);

	/* ----------------------- */
	/* Bind the point struct : */
	/* ----------------------- */
	define_spot_point_type<float, 3>("Point3f", spot_module);
	define_spot_point_type<double, 3>("Point3d", spot_module);

	/* ---------------------------------------------------------------------------------------- */
	/* Bind the basic chronometer and timing stats provided by the micro_benchmark.hpp header : */
	/* ---------------------------------------------------------------------------------------- */
	pybind11::class_<Stats>(spot_module, "TimeSeriesStatistics")
	    .def_property_readonly("mean", 					[&](const Stats& stats) { return coarse_to_time(stats.mean); })
		.def_property_readonly("min", 					[&](const Stats& stats) { return coarse_to_time(stats.min); })
		.def_property_readonly("max", 					[&](const Stats& stats) { return coarse_to_time(stats.max); })
		.def_property_readonly("variance", 				[&](const Stats& stats) { return coarse_to_time(stats.variance); })
		.def_property_readonly("standard_deviation", 	[&](const Stats& stats) { return coarse_to_time(stats.std_dev); })
		.def_property_readonly("quartile_first", 		[&](const Stats& stats) { return duration_to_time(stats.quartile_1); })
		.def_property_readonly("median", 				[&](const Stats& stats) { return duration_to_time(stats.median); })
		.def_property_readonly("quartile_third", 		[&](const Stats& stats) { return duration_to_time(stats.quartile_3); })
		.def_property_readonly("percentile_90", 		[&](const Stats& stats) { return duration_to_time(stats.percentile_90); })
		.def_property_readonly("percentile_95", 		[&](const Stats& stats) { return duration_to_time(stats.percentile_95); })
		.def_property_readonly("percentile_99", 		[&](const Stats& stats) { return duration_to_time(stats.percentile_99); })
		.def_property_readonly("total_running_time", 	[&](const Stats& stats) { return coarse_to_time(stats.total_running_time); })
		.doc() = "A simple structure to get some stats from a time series.";
	pybind11::class_<Timings>(spot_module, "TimingsLogger", pybind11::buffer_protocol())
	    .def(pybind11::init<unsigned int>(), "pre_allocated_laps"_a, pydoc("Allocates a timer with enough \"spots\" to keep all iteration times in memory"))
		.def("start", &Timings::start_lap, pydoc("Starts a lap on the timer."))
		.def("stop", &Timings::stop_lap, pydoc("Stops the current lap, and computes the time delta for this iteration."))
		.def("timings", [](const Timings& timings) {
			auto& stats = timings.get_time_statistics();
			return stats != nullptr ? *stats : micro_benchmarks::TimeSeriesStatistics();
		}, pydoc("Returns the computed statistics for this chronometer, if any."))
		.def("compute_stats", &Timings::compute_timing_stats, pydoc("Computes the timing statistics for this timer."))
		.def("print_timings", &Timings::print_timings,
				"banner_message"_a = "Timings for the current registration",
				"message_prefix"_a = "",
				pydoc("Prints the timing statistics with optional banner (once) and prefix (per-line) messages")
		)
		.def_buffer([](Timings& timings) -> pybind11::buffer_info {
			// Get original durations (c++ struct) and copy them as lengths (floating point units) in new buffer :
			auto times = timings.get_iteration_times();
			std::vector<micro_benchmarks::coarse_duration_t::rep> converted_duration(times.size());
			std::vector<micro_benchmarks::coarse_duration_t::rep>::size_type index = 0;
			// Convert durations one at a time and "collapse" them into a single value :
			for (auto& iteration_time : times) {
				converted_duration[index] = std::chrono::duration_cast<micro_benchmarks::coarse_duration_t>(iteration_time).count();
				index++;
			}
			/* Constructor requirements : (a) Data, (b) size of one scalar and (c) format descriptor (Python-like struct) (d) dimensions, */
			/* (e) size of each dimension, and (f) strides for each index. */
			return pybind11::buffer_info(
				/* a */ converted_duration.data(),
				/* b */ sizeof(micro_benchmarks::coarse_duration_t::rep),
				/* c */ pybind11::format_descriptor<micro_benchmarks::coarse_duration_t::rep>::format(),
				/* d */ 1,
				/* e */ { converted_duration.size() },
				/* f */ { sizeof(micro_benchmarks::coarse_duration_t::rep) }
			);
		})
		.def("__str__", [](const Timings& t) { t.print_timings("", ""); } )
		.doc() = "Simple interface to a small structure embedded in the FIST wrappers to keep track of iteration times.";


	/* --------------------------------------------------------- */
	/* --- Bind the FIST wrappers and their member functions --- */
	/* --------------------------------------------------------- */
	pybind11::class_<FISTBase>(spot_module, "FIST_BaseWrapper")
		//.def("get_timings", &FISTBase::get_timings)
		.def("lap_time", &FISTBase::get_running_time, "lap_number"_a)
		.def("print_timings", &FISTBase::print_timings, "message"_a = "", "prefix"_a = "")
		.def("set_max_iterations", &FISTBase::set_maximum_iterations, "max_iterations"_a = 200)
		.def("set_max_directions", &FISTBase::set_maximum_directions, "max_directions"_a = 100)
		.def_property_readonly("source_distribution", &FISTBase::get_source_point_cloud_py, pydoc("Return the source distribution."))
		.def_property_readonly("target_distribution", &FISTBase::get_target_point_cloud_py, pydoc("Return the target distribution."))
		.def_property_readonly("source_distribution_size", &FISTBase::get_source_distribution_size, pydoc("Return the size of source distribution."))
		.def_property_readonly("target_distribution_size", &FISTBase::get_target_distribution_size, pydoc("Return the size of target distribution."))
		.def_property_readonly("running_time", &FISTBase::get_total_running_time, pydoc("Get the total running time in seconds of the last computed registration run, or 0."))
		.def_property_readonly("matrix", &FISTBase::get_computed_matrix, pydoc("Returns the computed matrix for the two point clouds, or the identity matrix if not computed."))
		.def_property_readonly("translation", &FISTBase::get_computed_translation, pydoc("Returns the computed translation for the two point clouds, or a null vector if not computed."))
		.def_property_readonly("scaling", &FISTBase::get_computed_scaling, pydoc("Returns the computed scale between the two point clouds, or 1.0 if not computed."))
		.doc() = "This is the base class of all FIST wrappers. Defines some custom interface all wrappers abide by. "
				"It cannot be instantiated directly, look for one of the derived classes from this like FISTRandomPointClouds for example.";

	// With randomly generated distributions :
	pybind11::class_<FISTRandom, FISTBase>(spot_module, "FISTRandomPointClouds")
		.def(pybind11::init<std::uint32_t, std::uint32_t, double>(), "source_distribution_size"_a, "target_distribution_size"_a, "point_cloud_radius"_a = 1.0)
		.def("compute_transformation", &FISTRandom::compute_transformation, "enable_timings"_a = false)
		.def("__repr__", [](const FISTRandom& fist) {
			return fmt::format("<spot_wrappers::FISTWrapperRandomModels with {} and {} samples>", fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
		.doc() = "Generates random point clouds and registers them.";

	// With the same model and a transform :
	pybind11::class_<FISTSame, FISTBase>(spot_module, "FISTSamePointClouds")
		.def(pybind11::init<std::string>(), "source_model_path"_a)
		.def(pybind11::init<std::string, glm::mat3, glm::vec3>(), "source_model_path"_a, "transform"_a, "translation"_a)
		.def(pybind11::init<std::string, glm::mat3, glm::vec3, double>(), "source_model_path"_a, "transform"_a, "translation"_a, "scale"_a)
		.def("compute_transformation", &FISTSame::compute_transformation, "enable_timings"_a = false)
		.def_property_readonly("known_transform", &FISTSame::get_known_matrix, pydoc("Get the original matrix applied to the model."))
		.def_property_readonly("known_translation", &FISTSame::get_known_translation, pydoc("Get the original translation applied to the model."))
		.def_property_readonly("known_scaling", &FISTSame::get_known_scaling, pydoc("Get the original scale factor applied to the model."))
		.def("__repr__", [](const FISTSame& fist) {
			return fmt::format("<spot_wrappers::FISTWrapperRandomModels with {} and {} samples>", fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
		.doc() = "Loads one point cloud from an OFF file, applies a known transform and registers the two.";

	// With different models :
	pybind11::class_<FISTDifferent, FISTBase>(spot_module, "FISTDifferentPointClouds")
		.def(pybind11::init<const std::string&, const std::string&>(), "source_model_path"_a, "target_model_path"_a)
		.def("compute_transformation", &FISTDifferent::compute_transformation, "enable_timings"_a = false)
		.def("__repr__", [](const FISTDifferent& fist) {
			return fmt::format("<spot_wrappers::FISTWrapperRandomModels with {} and {} samples>", fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
		.doc() = "Loads two point clouds from OFF files and registers them.";
}
