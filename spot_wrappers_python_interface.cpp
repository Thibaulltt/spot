//
// Created by thibault on 21/09/22.
//

#include "spot_wrappers.hpp"

PYBIND11_MODULE(spot, spot_module) {
	// Those argument literals are __really__ useful ...
	using namespace pybind11::literals;

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

	/*
	 *py::class_<Matrix>(m, "Matrix", py::buffer_protocol())
		   .def_buffer([](Matrix &m) -> py::buffer_info {
				return py::buffer_info(
					m.data(),                               * Pointer to buffer *
					sizeof(float),                          * Size of one scalar *
					py::format_descriptor<float>::format(), * Python struct-style format descriptor *
					2,                                      * Number of dimensions *
					{ m.rows(), m.cols() },                 * Buffer dimensions *
					{ sizeof(float) * m.cols(),             * Strides (in bytes) for each index *
					  sizeof(float) }
			);
		});
	 */
	// Bind the point struct :
	pybind11::class_<Point<3, double>>(spot_module, "Point3d", pybind11::buffer_protocol())
		.def(pybind11::init<>())
		.def_buffer([](Point<3,double>& point) -> pybind11::buffer_info {
			return pybind11::buffer_info(
				point.get(),
				sizeof(double),
				pybind11::format_descriptor<double>::format(),
				1,
				{ 3 },
				{ sizeof(double) }
			);
		})
		.def("__repr__", [](const Point<3, double>& p) {
			return fmt::format("<interface to Point<3, float> : [ {} {} {} ]>", p[0], p[1], p[2]);
		});
	pybind11::class_<std::vector<Point<3, float>>>(spot_module, "vecPoint3f", pybind11::buffer_protocol())
		.def(pybind11::init<>())
		.def_buffer([](std::vector<Point<3, float>>& point) -> pybind11::buffer_info {
			return pybind11::buffer_info(
				point.data(),
				sizeof(Point<3, float>),
				pybind11::format_descriptor<Point<3, float>>::format(),
				2,
				{ static_cast<ssize_t>(point.size()), static_cast<ssize_t>(3) },
				{ sizeof(float) * 3, sizeof(float) }
			);
		});
	pybind11::class_<Point<3, float>>(spot_module, "Point3f", pybind11::buffer_protocol())
		.def(pybind11::init<>())
		.def_buffer([](Point<3,float>& point) -> pybind11::buffer_info {
			return pybind11::buffer_info(
				point.get(),
				sizeof(float),
				pybind11::format_descriptor<float>::format(),
				1,
				{ 3 },
				{ sizeof(float) }
			);
		})
		.def("__repr__", [](const Point<3, float>& p) {
			return fmt::format("<interface to Point<3, float> : [ {} {} {} ]>", p[0], p[1], p[2]);
		});

	using pydoc = pybind11::doc;

	// --- Bind the FIST wrappers and their member functions ---

	// With randomly generated distributions :
	using FISTRandom = spot_wrappers::FISTWrapperRandomModels;
	pybind11::class_<FISTRandom>(spot_module, "FISTRandomPointClouds")
		.def(pybind11::init<std::uint32_t, std::uint32_t, double>(),
			 "src_distribution_size"_a, "tgt_distribution_size"_a, "point_cloud_radius"_a,
			 pydoc("Generates two point clouds with a given radius.")
		)
		.def("matrix", &FISTRandom::get_transform_matrix, pydoc("Returns the computed matrix for the two point clouds, or the identity matrix if not computed."))
		.def("translation", &FISTRandom::get_transform_translation, pydoc("Returns the computed translation for the two point clouds, or a null vector if not computed."))
		.def("scaling", &FISTRandom::get_transform_scaling, pydoc("Returns the computed scale between the two point clouds, or 1.0 if not computed."))
		.def("lap_time", &FISTRandom::get_running_time, "lap_number"_a)
		.def("compute_transformation", &FISTRandom::compute_transformation, "enable_timings"_a = false)
		.def("print_timings", &FISTRandom::print_timings, "message"_a = "", "prefix"_a = "")
		.def("set_max_iterations", &FISTRandom::set_maximum_iterations, "max_iterations"_a = 200)
		.def("set_max_directions", &FISTRandom::set_maximum_directions, "max_directions"_a = 100)
		.def_property_readonly("source_distribution", &FISTRandom::get_source_point_cloud_py, pydoc("Return the source distribution."))
		.def_property_readonly("target_distribution", &FISTRandom::get_target_point_cloud_py, pydoc("Return the target distribution."))
		.def_property_readonly("running_time", &FISTRandom::get_total_running_time, pydoc("Get the total running time in seconds of the last registration run. Returns 0.0 otherwise."))
		.def("__repr__", [](const FISTRandom& fist) {
			return fmt::format("<interface to spot_wrappers::FISTWrapperRandomModels with {} and {} samples>",
							   fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
		.doc() = "Generates random point clouds and registers them.";

	// With the same model and a transform :
	using FISTSame = spot_wrappers::FISTWrapperSameModel;
	pybind11::class_<FISTSame>(spot_module, "FISTSamePointClouds")
		.def(pybind11::init<std::string>())
		.def(pybind11::init<std::string, glm::mat3, glm::vec3>())
		.def(pybind11::init<std::string, glm::mat3, glm::vec3, double>())
		.def("matrix", &FISTSame::get_transform_matrix, pydoc("Returns the computed matrix for the two point clouds, or the identity matrix if not computed."))
		.def("translation", &FISTSame::get_transform_translation, pydoc("Returns the computed translation for the two point clouds, or a null vector if not computed."))
		.def("scaling", &FISTSame::get_transform_scaling, pydoc("Returns the computed scale between the two point clouds, or 1.0 if not computed."))
		.def("lap_time", &FISTSame::get_running_time, "lap_number"_a)
		.def("compute_transformation", &FISTSame::compute_transformation, "enable_timings"_a = false)
		.def("print_timings", &FISTSame::print_timings, "message"_a = "", "prefix"_a = "")
		.def("set_max_iterations", &FISTSame::set_maximum_iterations, "max_iterations"_a = 200)
		.def("set_max_directions", &FISTSame::set_maximum_directions, "max_directions"_a = 100)
		.def_property_readonly("source_distribution", &FISTSame::get_source_point_cloud_py, pydoc("Return the source distribution."))
		.def_property_readonly("target_distribution", &FISTSame::get_target_point_cloud_py, pydoc("Return the target distribution."))
		.def_property_readonly("running_time", &FISTSame::get_total_running_time, pydoc("Get the total running time in seconds of the last registration run. Returns 0.0 otherwise."))
		.def_property_readonly("known_transform", &FISTSame::get_known_matrix, pydoc("Get the original matrix applied to the model."))
		.def_property_readonly("known_translation", &FISTSame::get_known_translation, pydoc("Get the original translation applied to the model."))
		.def_property_readonly("known_scaling", &FISTSame::get_known_scaling, pydoc("Get the original scale factor applied to the model."))
		.def("__repr__", [](const FISTSame& fist) {
			return fmt::format("<interface to spot_wrappers::FISTWrapperRandomModels with {} and {} samples>",
							   fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
		.doc() = "Loads one point cloud from an OFF file, applies a known transform and registers the two.";

	// With different models :
	using FISTDifferent = spot_wrappers::FISTWrapperDifferentModels;
	pybind11::class_<FISTDifferent>(spot_module, "FISTDifferentPointClouds")
		.def(pybind11::init<const std::string&, const std::string&>())
		.def("matrix", &FISTDifferent::get_transform_matrix, pydoc("Returns the computed matrix for the two point clouds, or the identity matrix if not computed."))
		.def("translation", &FISTDifferent::get_transform_translation, pydoc("Returns the computed translation for the two point clouds, or a null vector if not computed."))
		.def("scaling", &FISTDifferent::get_transform_scaling, pydoc("Returns the computed scale between the two point clouds, or 1.0 if not computed."))
		.def("lap_time", &FISTDifferent::get_running_time, "lap_number"_a)
		.def("compute_transformation", &FISTDifferent::compute_transformation, "enable_timings"_a = false)
		.def("print_timings", &FISTDifferent::print_timings, "message"_a = "", "prefix"_a = "")
		.def("set_max_iterations", &FISTDifferent::set_maximum_iterations, "max_iterations"_a = 200)
		.def("set_max_directions", &FISTDifferent::set_maximum_directions, "max_directions"_a = 100)
		.def_property_readonly("source_distribution", &FISTDifferent::get_source_point_cloud_py, pydoc("Return the source distribution."))
		.def_property_readonly("target_distribution", &FISTDifferent::get_target_point_cloud_py, pydoc("Return the target distribution."))
		.def_property_readonly("running_time", &FISTDifferent::get_total_running_time, pydoc("Get the total running time in seconds of the last registration run. Returns 0.0 otherwise."))
		.def("__repr__", [](const FISTDifferent& fist) {
			return fmt::format("<interface to spot_wrappers::FISTWrapperRandomModels with {} and {} samples>",
							   fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
		.doc() = "Loads two point clouds from OFF files and registers them.";
}
