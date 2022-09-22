//
// Created by thibault on 21/09/22.
//

#include "spot_wrappers.hpp"

PYBIND11_MODULE(spot, spot_module) {
	// Those argument literals are __really__ useful ...
	using namespace pybind11::literals;
	// Shorten the definition of documentation string literals :
	using pydoc = pybind11::doc;

	// Typedefs to the types to wrap :
	using FISTBase = spot_wrappers::FIST_BaseWrapper;
	using FISTRandom = spot_wrappers::FISTWrapperRandomModels;
	using FISTSame = spot_wrappers::FISTWrapperSameModel;
	using FISTDifferent = spot_wrappers::FISTWrapperDifferentModels;

	spot_module.doc() = "A set of wrappers around the SPOT method : from Sliced Partial Optimal Transport by Bonneel and Coeurjolly (2019)";

	// Declare used GLM types :
	define_glm_type_matrix(glm::mat4{}, "glm_mat4", spot_module);
	define_glm_type_matrix(glm::mat3{}, "glm_mat3", spot_module);
	define_glm_type_vector(glm::vec4{}, "glm_vec4", spot_module);
	define_glm_type_vector(glm::vec3{}, "glm_vec3", spot_module);

	spot_module.def("enable_reproducible_runs", [](){ spot_wrappers::set_enable_reproducible_runs(true); })
		.doc() = "Enables reproducible runs : sets the random engine to be initialized with a constant value instead of a timestamp.";
	spot_module.def("disable_reproducible_runs", [](){ spot_wrappers::set_enable_reproducible_runs(false); })
		.doc() = "Disables reproducible runs : sets the random engine to be initialized with a timestamp instead of a constant value.";

	// Bind the point struct :

	pybind11::class_<Point<3, double>>(spot_module, "Point3d", pybind11::buffer_protocol())
		.def(pybind11::init<>())
		.def_buffer([](Point<3,double>& point) -> pybind11::buffer_info {
			return pybind11::buffer_info(point.get(), sizeof(double),
				pybind11::format_descriptor<double>::format(),1, { 3 }, { sizeof(double) }
			);
		})
		.def("__repr__", [](const Point<3, double>& p) {
			return fmt::format("<interface to Point<3, float> : [ {} {} {} ]>", p[0], p[1], p[2]);
		});
	pybind11::class_<Point<3, float>>(spot_module, "Point3f", pybind11::buffer_protocol())
		.def(pybind11::init<>())
		.def_buffer([](Point<3,float>& point) -> pybind11::buffer_info {
			return pybind11::buffer_info(point.get(), sizeof(float),
				pybind11::format_descriptor<float>::format(), 1, { 3 }, { sizeof(float) }
			);
		})
		.def("__repr__", [](const Point<3, float>& p) {
			return fmt::format("<interface to Point<3, float> : [ {} {} {} ]>", p[0], p[1], p[2]);
		});


	// --- Bind the FIST wrappers and their member functions ---
	pybind11::class_<FISTBase>(spot_module, "FIST_BaseWrapper")
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
		.doc() = "";

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
