#include "./spot_wrappers.hpp"
#include "../external/fmt_bridge.hpp"

namespace spot_wrappers {

	void set_enable_reproducible_runs(bool _enable) {
		fmtdbg("Modifying reproducibility to {}", _enable);
		enable_reproducible_runs = _enable;
		initialize_random_engines();
	}

	void initialize_random_engines() {
		if (enable_reproducible_runs) {
			fmtdbg("Initializing random to sequential !");
			engine = std::default_random_engine(10);
		} else {
			/* Init with current time : */
			fmtdbg("Initializing random with timestamp !");
			auto current_time = std::chrono::high_resolution_clock::now();
			engine = std::default_random_engine(current_time.time_since_epoch().count());
		}
	}

	point_tensor_t point_vector_to_tensor(const std::vector<Point<3, float>>& source) {
		return point_tensor_t(
			pybind11::array::ShapeContainer({static_cast<ssize_t>(source.size()), static_cast<ssize_t>(3)}),
			pybind11::array::StridesContainer({sizeof(float), sizeof(float) * 3}),
			reinterpret_cast<float*>(const_cast<Point<3, float>*>(source.data())),
			pybind11::array::handle()
		);
	}
	//endregion

	//region --- FIST_BaseWrapper implementation ---
	FIST_BaseWrapper::FIST_BaseWrapper() :
		computed_transform(glm::identity<glm::mat4>()), computed_translation({}), computed_scaling(1.0)
	{
		fmtdbg("FIST_BaseWrapper::ctor()");
		this->timings = nullptr;
		this->maximum_iterations = 200;
		this->maximum_directions = 100;
	}

	FIST_BaseWrapper::~FIST_BaseWrapper() {
		fmtdbg("FIST_BaseWrapper::dtor()");
		this->timings.reset();
	}

	point_tensor_t FIST_BaseWrapper::get_source_point_cloud_py() const {
		return point_vector_to_tensor(this->get_source_distribution());
	}

	point_tensor_t FIST_BaseWrapper::get_target_point_cloud_py() const {
		return point_vector_to_tensor(this->get_target_distribution());
	}

	std::uint32_t FIST_BaseWrapper::get_source_distribution_size() const {
		return static_cast<std::uint32_t>(this->get_source_distribution().size());
	}

	std::uint32_t FIST_BaseWrapper::get_target_distribution_size() const {
		return static_cast<std::uint32_t>(this->get_target_distribution().size());
	}

	double FIST_BaseWrapper::get_total_running_time() const {
		fmtdbg("FIST_BaseWrapper::get_total_running_time() : Timings is {}", static_cast<void*>(this->timings.get()));
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

	double FIST_BaseWrapper::get_running_time(std::uint32_t lap_number) const {
		fmtdbg("FIST_BaseWrapper::get_running_time({}) : Timings is {}", lap_number, static_cast<void*>(this->timings.get()));
		if (this->timings) {
			if (lap_number >= this->timings->get_iteration_times().size()) {
				return 0.0;
			} else {
				return std::chrono::duration_cast<micro_benchmarks::fine_duration_t>(this->timings->get_iteration_times()[lap_number]).count();
			}
		} else return 0.0;
	}

	const micro_benchmarks::TimingsLogger FIST_BaseWrapper::get_timings() const {
		if (this->timings) {
			micro_benchmarks::TimingsLogger benchmarks = *(this->timings);
			return {benchmarks};
		} else {
			return micro_benchmarks::TimingsLogger();
		}
	}

	void FIST_BaseWrapper::print_timings(const char* message, const char* prefix) const {
		if (this->timings) {
			this->timings->print_timings(message, prefix);
		} else {
			std::cerr << "<Error : no timings recorded>\n";
		}
	}

	void FIST_BaseWrapper::set_maximum_iterations(const std::uint32_t new_iterations_max) {
		fmtdbg("FIST_BaseWrapper::set_maximum_iterations() : setting {} to {}", this->maximum_iterations, new_iterations_max);
		this->maximum_iterations = new_iterations_max;
	}

	void FIST_BaseWrapper::set_maximum_directions(const std::uint32_t new_directions_max) {
		fmtdbg("FIST_BaseWrapper::set_maximum_directions() : setting {} to {}", this->maximum_directions, new_directions_max);
		this->maximum_directions = new_directions_max;
	}

	glm::mat4 FIST_BaseWrapper::get_computed_matrix() const {
		return this->computed_transform;
	}

	glm::vec4 FIST_BaseWrapper::get_computed_translation() const {
		return this->computed_translation;
	}

	double FIST_BaseWrapper::get_computed_scaling() const {
		return this->computed_scaling;
	}
	//endregion

	//region --- FISTWrapperRandomModels implementation ---
	FISTWrapperRandomModels::FISTWrapperRandomModels(std::uint32_t src_distrib_size, std::uint32_t tgt_distrib_size, double radius) :
		src_size(src_distrib_size), tgt_size(tgt_distrib_size), point_cloud_radius(radius),
		source_distribution(src_distrib_size), target_distribution(tgt_distrib_size), FIST_BaseWrapper()
	{
		fmtdbg("FISTWrapperRandomModels::ctor({}, {}, {})", this->src_size, this->tgt_size, this->point_cloud_radius);
		// Generate random point clouds :
		for (std::size_t i = 0; i < static_cast<std::size_t>(this->src_size); ++i) {
			this->source_distribution[i][0] = static_cast<float>(uniform(engine) * this->point_cloud_radius);
			this->source_distribution[i][1] = static_cast<float>(uniform(engine) * this->point_cloud_radius);
			this->source_distribution[i][2] = static_cast<float>(uniform(engine) * this->point_cloud_radius);
		}
		for (std::size_t i = 0; i < static_cast<std::size_t>(this->tgt_size); ++i) {
			this->target_distribution[i][0] = static_cast<float>((uniform(engine) + 2.0 + 2) * this->point_cloud_radius);
			this->target_distribution[i][1] = static_cast<float>((uniform(engine) + 2.0 + 4) * this->point_cloud_radius);
			this->target_distribution[i][2] = static_cast<float>((uniform(engine) + 2.0 + 6) * this->point_cloud_radius);
		}
	}

	FISTWrapperRandomModels::~FISTWrapperRandomModels() = default;

	void FISTWrapperRandomModels::compute_transformation(bool enable_timings) {
		fmtdbg("FISTWrapperRandomModels::compute_transformation({})", enable_timings);
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
		fmt::print("Registration done.\n");
		if (enable_timings) {
			this->timings->print_timings(
				fmt::format("After registering {} to {} points, transformation is :", this->src_size, this->tgt_size),
				"[Final transformation :]");
		}
	}

	std::vector<Point<3, float>>& FISTWrapperRandomModels::get_source_distribution() {
		return this->source_distribution;
	}

	const std::vector<Point<3, float>>& FISTWrapperRandomModels::get_source_distribution() const {
		return this->source_distribution;
	}

	std::vector<Point<3, float>>& FISTWrapperRandomModels::get_target_distribution() {
		return this->target_distribution;
	}

	const std::vector<Point<3, float>>& FISTWrapperRandomModels::get_target_distribution() const {
		return this->target_distribution;
	}
	//endregion

	//region --- FISTWrapperSameModel implementation ---
	FISTWrapperSameModel::FISTWrapperSameModel(std::string src_path) :
		known_transform(glm::identity<glm::mat3>()), known_translation({}), known_scaling(1.f),
		source_model_path(std::move(src_path)), source_model(nullptr), target_model(nullptr), FIST_BaseWrapper()
	{
		fmtdbg("FISTWrapperSameModel::ctor({})", src_path);
		this->initialize_and_transform_models();
	}

	FISTWrapperSameModel::FISTWrapperSameModel(std::string src_path, glm::mat3 rotation, glm::vec3 translation) :
		known_transform(rotation), known_scaling(1.f), known_translation(translation, 0.0f),
		source_model_path(std::move(src_path)), source_model(nullptr), target_model(nullptr), FIST_BaseWrapper()
	{
		fmtdbg("FISTWrapperSameModel::ctor({}, mat3, vec3)", src_path);
		this->initialize_and_transform_models();
	}

	FISTWrapperSameModel::FISTWrapperSameModel(std::string src_path, glm::mat3 rotation, glm::vec3 translation, double scale) :
		known_transform(rotation), known_scaling(scale), known_translation(translation, 0.0f),
		source_model_path(std::move(src_path)), source_model(nullptr), target_model(nullptr), FIST_BaseWrapper()
	{
		fmtdbg("FISTWrapperSameModel::ctor({}, mat3, vec3, {})", src_path, scale);
		this->initialize_and_transform_models();
	}

	void FISTWrapperSameModel::initialize_and_transform_models() {
		fmtdbg("Loading model at \"{}\" ...", this->source_model_path);
		this->source_model = std::make_unique<Model>(std::move(load_off_file(this->source_model_path)));
		this->target_model = std::make_unique<Model>(std::cref(*this->source_model)); // cref -> allows to force copy instead of move ?
		fmtdbg("Loaded and copied.", this->source_model_path);
		this->target_model->apply_scaling(this->known_scaling);
		this->target_model->apply_transform(this->known_transform);
		this->target_model->apply_translation(this->known_translation);
		fmtdbg("Applied transformation");
	}

	FISTWrapperSameModel::~FISTWrapperSameModel() = default;

	void FISTWrapperSameModel::compute_transformation(bool enable_timings) {
		fmtdbg("FISTWrapperSameModel::compute_transformation()");
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
			this->source_model->positions, this->target_model->positions,
			rot, trans, false, scaling, std::move(this->timings)
		);
		this->computed_transform = glm::mat4{
			rot[0], rot[1], rot[2], 0.0f,
			rot[3], rot[4], rot[5], 0.0f,
			rot[6], rot[7], rot[8], 0.0f,
			0.0f,   0.0f,   0.0f,   1.0f
		};
		fmtdbg("Final transform : {}", rot);
		fmtdbg("Final translation : {}", trans);
		this->computed_translation = glm::vec4(trans[0], trans[1], trans[2], 0.0f);
		this->computed_scaling = scaling;
		fmt::print("Registration done.\n");
		if (enable_timings) {
			this->timings->print_timings(
				fmt::format("After registering {} to {} points, transformation is :",
							this->source_model->positions.size(), this->target_model->positions.size()),
				"[Final transformation :]");
		}
	}

	glm::mat4 FISTWrapperSameModel::get_known_matrix() const {
		return this->known_transform;
	}

	glm::vec4 FISTWrapperSameModel::get_known_translation() const {
		return this->known_translation;
	}

	double FISTWrapperSameModel::get_known_scaling() const {
		return this->known_scaling;
	}

	std::vector<Point<3, float>>& FISTWrapperSameModel::get_source_distribution() {
		return this->source_model->positions;
	}

	const std::vector<Point<3, float>>& FISTWrapperSameModel::get_source_distribution() const {
		return this->source_model->positions;
	}

	std::vector<Point<3, float>>& FISTWrapperSameModel::get_target_distribution() {
		return this->target_model->positions;
	}

	const std::vector<Point<3, float>>& FISTWrapperSameModel::get_target_distribution() const {
		return this->target_model->positions;
	}
	//endregion

	//region --- FISTWrapperDifferentModels implementation ---
	FISTWrapperDifferentModels::FISTWrapperDifferentModels(std::string src_path, std::string tgt_path) :
		source_file_path(std::move(src_path)), target_file_path(std::move(tgt_path)), FIST_BaseWrapper()
	{
		this->source_model = std::make_unique<Model>(load_off_file(this->source_file_path));
		this->target_model = std::make_unique<Model>(load_off_file(this->target_file_path));
	}

	FISTWrapperDifferentModels::~FISTWrapperDifferentModels() = default;

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
			this->source_model->positions, this->target_model->positions,
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
		fmt::print("Registration done.\n");
		if (enable_timings) {
			this->timings->print_timings(
				fmt::format("After registering {} to {} points, transformation is :",
							this->source_model->positions.size(), this->target_model->positions.size()),
				"[Final transformation :]");
		}
	}

	std::vector<Point<3, float>>& FISTWrapperDifferentModels::get_source_distribution() {
		return this->source_model->positions;
	}

	const std::vector<Point<3, float>>& FISTWrapperDifferentModels::get_source_distribution() const {
		return this->source_model->positions;
	}

	std::vector<Point<3, float>>& FISTWrapperDifferentModels::get_target_distribution() {
		return this->target_model->positions;
	}

	const std::vector<Point<3, float>>& FISTWrapperDifferentModels::get_target_distribution() const {
		return this->target_model->positions;
	}
	//endregion

}
