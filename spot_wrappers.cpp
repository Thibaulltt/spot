#include "./spot_wrappers.hpp"
#include "glm_bridge.hpp"

#include <pybind11/attr.h>

namespace spot_wrappers {

	void set_enable_reproducible_runs(bool _enable) {
		enable_reproducible_runs = _enable;
		initialize_random_engines();
	}

	void initialize_random_engines() {
		if (enable_reproducible_runs) {
			engine = std::default_random_engine(10);
		} else {
			/* Init with current time : */
			auto current_time = std::chrono::high_resolution_clock::now();
			engine = std::default_random_engine(current_time.time_since_epoch().count());
		}
	}

	//region --- Spot_BaseWrapper implementation ---
	SPOT_BaseWrapper::SPOT_BaseWrapper() {
		this->timings = nullptr;
		this->maximum_iterations = 200;
		this->maximum_directions = 100;
	}

	SPOT_BaseWrapper::~SPOT_BaseWrapper() {
		this->timings.reset();
	}

	double SPOT_BaseWrapper::get_total_running_time() const {
		if (this->timings) {
			auto times = this->timings->get_iteration_times();
			return std::accumulate(times.cbegin(), times.cend(), 0.0,
				[&](double before, const micro_benchmarks::duration_t& duration) {
					// Convert to a duration type with double (fp64) precision, and add its count to the current sum :
					return before + std::chrono::duration_cast<micro_benchmarks::fine_duration_t>(duration).count();
				}
			);
		} else return 0.0;
	}

	double SPOT_BaseWrapper::get_running_time(std::uint32_t lap_number) const {
		if (this->timings) {
			if (lap_number >= this->timings->get_iteration_times().size()) {
				return 0.0;
			} else {
				return std::chrono::duration_cast<micro_benchmarks::fine_duration_t>(this->timings->get_iteration_times()[lap_number]).count();
			}
		} else return 0.0;
	}

	void SPOT_BaseWrapper::print_timings(const char* message, const char* prefix) const {
		if (this->timings) {
			this->timings->print_timings(message, prefix);
		} else {
			std::cerr << "<Error : no timings recorded>\n";
		}
	}

	void SPOT_BaseWrapper::set_maximum_iterations(const std::uint32_t new_iterations_max) {
		this->maximum_iterations = new_iterations_max;
	}

	void SPOT_BaseWrapper::set_maximum_directions(const std::uint32_t new_directions_max) {
		this->maximum_directions = new_directions_max;
	}
	//endregion

	//region --- FISTWrapperRandomModels implementation ---
	FISTWrapperRandomModels::FISTWrapperRandomModels(std::uint32_t src_distrib_size, std::uint32_t tgt_distrib_size, double radius)
		: src_size(src_distrib_size), tgt_size(tgt_distrib_size), point_cloud_radius(radius),
		computed_transform(glm::identity<glm::mat4>()), computed_translation(glm::vec4{}), computed_scaling(1.0),
		source_distribution(src_distrib_size), target_distribution(tgt_distrib_size), SPOT_BaseWrapper()
	{
		// Generate random point clouds :
		for (std::size_t i = 0; i < this->src_size; ++i) {
			this->source_distribution[i][0] = uniform(engine) * this->point_cloud_radius;
			this->source_distribution[i][1] = uniform(engine) * this->point_cloud_radius;
			this->source_distribution[i][2] = uniform(engine) * this->point_cloud_radius;
		}
		for (std::size_t i = 0; i < this->tgt_size; ++i) {
			this->target_distribution[i][0] = uniform(engine) * this->point_cloud_radius;
			this->target_distribution[i][1] = uniform(engine) * this->point_cloud_radius;
			this->target_distribution[i][2] = uniform(engine) * this->point_cloud_radius;
		}
	}

	FISTWrapperRandomModels::~FISTWrapperRandomModels() = default;

	void FISTWrapperRandomModels::compute_transformation(bool enable_timings) {
		UnbalancedSliced sliced;
		std::vector<double> rot(9);
		std::vector<double> trans(3);
		double scaling;
		if (enable_timings) {
			this->timings = std::make_unique<micro_benchmarks::TimingsLogger>(this->maximum_iterations);
		}
		this->timings = sliced.fast_iterative_sliced_transport(
			static_cast<int>(this->maximum_iterations),
			static_cast<int>(this->maximum_directions),
			this->source_distribution, this->target_distribution,
			rot, trans, true, scaling, std::move(this->timings)
		);
		this->computed_transform = glm::mat4{
			rot[0], rot[1], rot[2], 0.0f,
			rot[3], rot[4], rot[5], 0.0f,
			rot[6], rot[7], rot[8], 0.0f,
			0.0f,   0.0f,   0.0f,   1.0f
		};
		this->computed_translation = glm::vec4(trans[0], trans[1], trans[2], 0.0f);
		this->computed_scaling = scaling;
		fmt::print("Registration done.");
		if (enable_timings) {
			this->timings->compute_timing_stats();
			this->timings->print_timings(
				fmt::format("After registering {} to {} points, transformation is :", this->src_size, this->tgt_size),
				"[Final transformation :]");
		}
	}

	point_tensor_t FISTWrapperRandomModels::get_source_point_cloud_py() const {
		return pybind11::array_t<Point<3, float>, pybind11::array::c_style | pybind11::array::forcecast>(this->src_size, this->source_distribution.data());
	}

	point_tensor_t FISTWrapperRandomModels::get_target_point_cloud_py() const {
		return pybind11::array_t<Point<3, float>, pybind11::array::c_style | pybind11::array::forcecast>(this->tgt_size, this->target_distribution.data());
	}

	glm::mat4 FISTWrapperRandomModels::get_transform_matrix() const {
		return this->computed_transform;
	}

	glm::vec4 FISTWrapperRandomModels::get_transform_translation() const {
		return this->computed_translation;
	}

	double FISTWrapperRandomModels::get_transform_scaling() const {
		return this->computed_scaling;
	}

	std::uint32_t FISTWrapperRandomModels::get_source_distribution_size() const {
		return this->src_size;
	}

	std::uint32_t FISTWrapperRandomModels::get_target_distribution_size() const {
		return this->tgt_size;
	}
	//endregion

	//region --- FISTWrapperDifferentModels implementation ---
	FISTWrapperDifferentModels::FISTWrapperDifferentModels(const std::string src_path, const std::string tgt_path) :
	source_file_path(src_path), target_file_path(tgt_path),SPOT_BaseWrapper()
	{
		this->source_model = load_off_file(this->source_file_path);
		this->target_model = load_off_file(this->target_file_path);
	}

	FISTWrapperDifferentModels::~FISTWrapperDifferentModels() noexcept = default;

	void FISTWrapperDifferentModels::compute_transformation(bool enable_timings) {
		UnbalancedSliced sliced;
		std::vector<double> rot(9);
		std::vector<double> trans(3);
		double scaling;
		if (enable_timings) {
			this->timings = std::make_unique<micro_benchmarks::TimingsLogger>(this->maximum_iterations);
		}
		this->timings = sliced.fast_iterative_sliced_transport(
			static_cast<int>(this->maximum_iterations),
			static_cast<int>(this->maximum_directions),
			this->source_model.positions, this->target_model.positions,
			rot, trans, true, scaling, std::move(this->timings)
		);
		this->computed_transform = glm::mat4{
			rot[0], rot[1], rot[2], 0.0f,
			rot[3], rot[4], rot[5], 0.0f,
			rot[6], rot[7], rot[8], 0.0f,
			0.0f,   0.0f,   0.0f,   1.0f
		};
		this->computed_translation = glm::vec4(trans[0], trans[1], trans[2], 0.0f);
		this->computed_scaling = scaling;
		fmt::print("Registration done.");
		if (enable_timings) {
			this->timings->compute_timing_stats();
			this->timings->print_timings(
				fmt::format("After registering {} to {} points, transformation is :",
							this->source_model.positions.size(), this->target_model.positions.size()),
				"[Final transformation :]");
		}
	}

	point_tensor_t FISTWrapperDifferentModels::get_source_point_cloud_py() const {
		return pybind11::array_t<Point<3, float>, pybind11::array::c_style | pybind11::array::forcecast>(
			this->source_model.positions.size(), this->source_model.positions.data());
	}

	point_tensor_t FISTWrapperDifferentModels::get_target_point_cloud_py() const {
		return pybind11::array_t<Point<3, float>, pybind11::array::c_style | pybind11::array::forcecast>(
			this->target_model.positions.size(), this->target_model.positions.data());
	}

	glm::mat4 FISTWrapperDifferentModels::get_transform_matrix() const {
		return this->computed_transform;
	}

	glm::vec4 FISTWrapperDifferentModels::get_transform_translation() const {
		return this->computed_translation;
	}

	double FISTWrapperDifferentModels::get_transform_scaling() const {
		return this->computed_scaling;
	}

	std::uint32_t FISTWrapperDifferentModels::get_source_distribution_size() const {
		return static_cast<std::uint32_t>(this->source_model.positions.size());
	}

	std::uint32_t FISTWrapperDifferentModels::get_target_distribution_size() const {
		return static_cast<std::uint32_t>(this->target_model.positions.size());
	}
	//endregion

}

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

	/// @brief simple typedef, to make the following lines easier to read :
	using FISTRandom = spot_wrappers::FISTWrapperRandomModels;

	// Bind the point struct :
	pybind11::class_<Point<3, double>>(spot_module, "Point3d", pybind11::buffer_protocol())
		.def_buffer([](Point<3,double>& point){
			return pybind11::buffer_info( &point[0], 3 * sizeof(double),
				pybind11::format_descriptor<double>::format(), 1, { 3 }, { sizeof(double) }
			);
		});

	// Binds the FIST wrapper with random distributions and its member functions :
	pybind11::class_<FISTRandom>(spot_module, "FISTRandomPointClouds")
		.def(pybind11::init<const std::uint32_t, const std::uint32_t, const double>())
		.def("matrix", &FISTRandom::get_transform_matrix, pybind11::doc("Matrix documentation :)"))
		.def("translation", &FISTRandom::get_transform_translation)
		.def("scaling", &FISTRandom::get_transform_scaling)
		.def("lap_time", &FISTRandom::get_running_time, "lap_number"_a)
		.def("compute_transformation", &FISTRandom::compute_transformation, "enable_timings"_a = false)
		.def("print_timings", &FISTRandom::print_timings, "message"_a = "", "prefix"_a = "")
		.def("set_max_iterations", &FISTRandom::set_maximum_iterations, "max_iterations"_a = 200)
		.def("set_max_directions", &FISTRandom::set_maximum_directions, "max_directions"_a = 100)
		.def_property_readonly("source_distribution", &FISTRandom::get_source_point_cloud_py)
		.def_property_readonly("target_distribution", &FISTRandom::get_target_point_cloud_py)
		.def_property_readonly("running_time", &FISTRandom::get_total_running_time)
		.def("__repr__", [](const FISTRandom& fist) {
			return fmt::format("<interface to spot_wrappers::FISTWrapperRandomModels with {} and {} samples>",
					fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
	    .doc() = "Generates random point clouds and registers them.";

	using FISTDifferent = spot_wrappers::FISTWrapperDifferentModels;
	pybind11::class_<FISTDifferent>(spot_module, "FISTDifferentPointClouds")
	    .def(pybind11::init<const std::string&, const std::string&>())
		.def("matrix", &FISTDifferent::get_transform_matrix, pybind11::doc("Matrix documentation :)"))
		.def("translation", &FISTDifferent::get_transform_translation)
		.def("scaling", &FISTDifferent::get_transform_scaling)
		.def("lap_time", &FISTDifferent::get_running_time, "lap_number"_a)
		.def("compute_transformation", &FISTDifferent::compute_transformation, "enable_timings"_a = false)
		.def("print_timings", &FISTDifferent::print_timings, "message"_a = "", "prefix"_a = "")
		.def("set_max_iterations", &FISTDifferent::set_maximum_iterations, "max_iterations"_a = 200)
		.def("set_max_directions", &FISTDifferent::set_maximum_directions, "max_directions"_a = 100)
		.def_property_readonly("source_distribution", &FISTDifferent::get_source_point_cloud_py)
		.def_property_readonly("target_distribution", &FISTDifferent::get_target_point_cloud_py)
		.def_property_readonly("running_time", &FISTDifferent::get_total_running_time)
		.def("__repr__", [](const FISTDifferent& fist) {
			return fmt::format("<interface to spot_wrappers::FISTWrapperRandomModels with {} and {} samples>",
							   fist.get_source_distribution_size(), fist.get_target_distribution_size());
		})
		.doc() = "Loads two point clouds from OFF files and registers them.";
}