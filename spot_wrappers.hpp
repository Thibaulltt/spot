#ifndef SPOT__SPOT_WRAPPERS_HPP_
#define SPOT__SPOT_WRAPPERS_HPP_

/*=============================================
 * Creator     : thib
 * Created on  : 16/09/22
 * Path        : /spot_wrappers.hpp
 * Description : Some wrappers around the SPOT method, to make it easier to create Python bindings.
 *=============================================
 */

#include "./micro_benchmark.hpp"
#include "./UnbalancedSliced.h"
#include "./model.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <vector>

/// @brief This namespace englobes some wrappers around the SPOT method, in order to benchmark it.
namespace spot_wrappers {

	/// @brief Simple typedef to an array type containing the points' data.
	using point_tensor_t = pybind11::array_t<Point<3, float>, pybind11::array::c_style | pybind11::array::forcecast>;

	/// @brief Controls whether the random engine used to generate random numbers is const-initialized or time-initialized.
	static bool enable_reproducible_runs = false;

	/// @brief Enables or disables the ``enable_reproducible_runs`` variable, and re-initializes the random engines.
	void set_enable_reproducible_runs(bool _enable);

	/// @brief Initializes the random engines in the UnbalancedSliced.h file.
	/// @details If spot_wrappers::enable_reproducible_runs is true, assigns a constant to the engine initialization. Otherwise,
	/// will initialize the engines based on the current time duration since the UNIX epoch.
	void initialize_random_engines();

	/// @brief Base class for the wrappers around the SPOT method.
	/// @note This class is not meant to be created directly. Its sub-classes are.
	class SPOT_BaseWrapper {
	protected:
		/// @brief Default ctor for the base wrapper class.
		SPOT_BaseWrapper();

	public:
		/// @brief Default dtor for the base wrapper class.
		virtual ~SPOT_BaseWrapper();

		/// @brief Launches the SPOT method.
		/// @param enable_timings Whether to enable benchmark timings for the current run.
		virtual void compute_transformation(bool enable_timings) = 0;

		/// @brief Gets the source distribution data.
		virtual point_tensor_t get_source_point_cloud_py() const = 0;
		/// @brief Gets the target distribution data.
		virtual point_tensor_t get_target_point_cloud_py() const = 0;

		/// @brief Gets the total running time of the method.
		/// @returns The sum of all lap times for the last run of the SPOT method, or 0 if no timer was previously used.
		double get_total_running_time() const;
		/// @brief Gets the time that a specific iteration lasted for.
		/// @param lap_number The lap number to ask for. If over the number of laps, returns 0.
		/// @returns The running time of the `lap_number` lap, or 0 if the timer hasn't been used.
		double get_running_time(std::uint32_t lap_number) const;

		/// @brief Prints the timings computed, if available.
		void print_timings(const char* message, const char* prefix) const;

		/// @brief Sets the new maximum number of iterations available for registrations.
		void set_maximum_iterations(std::uint32_t new_iterations_max);
		/// @brief Sets the new maximum number of directions evaluated at each iteration of the registration.
		void set_maximum_directions(std::uint32_t new_directions_max);

		/// @brief Returns the size of the source distribution. Used for information in Python's ``__repr__`` function.
		virtual std::uint32_t get_source_distribution_size() const = 0;
		/// @brief Returns the size of the target distribution. Used for information in Python's ``__repr__`` function.
		virtual std::uint32_t get_target_distribution_size() const = 0;

	protected:
		std::unique_ptr<micro_benchmarks::TimingsLogger> timings; ///< The benchmark logger, to keep track of the execution times.

		std::uint32_t maximum_iterations; ///< The maximum number of iterations available for registration steps.
		std::uint32_t maximum_directions; ///< The maximum number of directions evaluated at each registration step.
	};

	/// @brief This wrapper for the FIST method generates random point clouds and registers them.
	class FISTWrapperRandomModels : public SPOT_BaseWrapper {
	public:
		/// @brief Ctor for a FIST wrapper registering two random point clouds.
		FISTWrapperRandomModels(std::uint32_t src_distrib_size, std::uint32_t tgt_distrib_size, double radius);
		/// @brief Default dtor for the random point cloud registration.
		~FISTWrapperRandomModels() override;

		/// @brief Computes the transformation between the two random point clouds.
		void compute_transformation(bool enable_timings) override;

		/// @brief Gets the source distribution data.
		point_tensor_t get_source_point_cloud_py() const override;
		/// @brief Gets the target distribution data.
		point_tensor_t get_target_point_cloud_py() const override;

		/// @brief Gets the currently computed rotation/scale matrix.
		/// @returns Either a identity matrix if it has not been computed, or the computed matrix.
		glm::mat4 get_transform_matrix() const;
		/// @brief Gets the currently computed translation.
		glm::vec4 get_transform_translation() const;
		/// @brief Returns the computed scaling parameter if requested.
		double get_transform_scaling() const;

		/// @brief Returns the size of the source distribution. Used for information in Python's ``__repr__`` function.
		std::uint32_t get_source_distribution_size() const override;
		/// @brief Returns the size of the target distribution. Used for information in Python's ``__repr__`` function.
		std::uint32_t get_target_distribution_size() const override;

	protected:
		glm::mat4 computed_transform;
		glm::vec4 computed_translation;
		double computed_scaling;

		std::uint32_t src_size;
		std::uint32_t tgt_size;
		double point_cloud_radius;

		std::vector<Point<3, float>> source_distribution; ///< The source point cloud.
		std::vector<Point<3, float>> target_distribution; ///< The target point cloud.
	};

	/// @brief This wrapper for the FIST method loads a model, copies it, and applies a known transform before registering them.
	class FISTWrapperSameModel : public SPOT_BaseWrapper {
	public:
		/// @brief Creates a FIST wrapper registering a model to its copy transformed with a random (rigid) transform.
		/// @param src_path The source path to the file to load.
		FISTWrapperSameModel(std::string const& src_path);
		/// @brief Creates a FIST wrapper registering a model to its copy transformed with a given (rigid) transform.
		/// @param src_path The source path to the file to load.
		/// @param rotation The rotation matrix to apply to the model.
		/// @param translation The translation vector to apply to the model.
		/// @note No checks are performed to see if the given rotation matrix is rigid.
		FISTWrapperSameModel(std::string const& src_path, glm::mat3 const& rotation, glm::vec3 const& translation);
		/// @brief Creates a FIST wrapper registering a model to its copy transformed with a given (similarity) transform.
		/// @param src_path The source path to the file to load.
		/// @param rotation The rotation matrix to apply to the model.
		/// @param translation The translation vector to apply to the model.
		/// @param scale The isotropic scale parameter to apply to the model.
		/// @note No checks are performed to see if the given rotation matrix is rigid.
		FISTWrapperSameModel(std::string const& src_path, glm::mat3 const& rotation, glm::vec3 const& translation, double const& scale);
		/// @brief Default dtor of the class.
		~FISTWrapperSameModel() override;

		/// @brief Gets the currently computed rotation/scale matrix.
		/// @returns Either a identity matrix if it has not been computed, or the computed matrix.
		glm::mat4 get_transform_matrix() const;
		/// @brief Gets the currently computed translation.
		glm::vec4 get_transform_translation() const;
		/// @brief Returns the computed scaling parameter if requested.
		double get_transform_scaling() const;

		/// @brief Gets the source distribution data.
		point_tensor_t get_source_point_cloud_py() const override;
		/// @brief Gets the target distribution data.
		point_tensor_t get_target_point_cloud_py() const override;

		/// @brief Returns the size of the source distribution. Used for information in Python's ``__repr__`` function.
		std::uint32_t get_source_distribution_size() const override;
		/// @brief Returns the size of the target distribution. Used for information in Python's ``__repr__`` function.
		std::uint32_t get_target_distribution_size() const override;

	protected:
		std::string source_model_path;

		glm::mat4 known_transform;
		glm::vec4 known_translation;
		double known_scaling;

		glm::mat4 computed_transform;
		glm::vec4 computed_translation;
		double computed_scaling;

		Model source_model;
		Model target_model;
	};

	/// @brief This wrapper for the FIST method loads two different models and registers them together.
	class FISTWrapperDifferentModels : public SPOT_BaseWrapper {
	public:
		FISTWrapperDifferentModels(std::string, std::string);
		~FISTWrapperDifferentModels() override;

		/// @brief Computes the transformation between the two models.
		void compute_transformation(bool enable_timings) override;

		/// @brief Gets the currently computed rotation/scale matrix.
		/// @returns Either a identity matrix if it has not been computed, or the computed matrix.
		glm::mat4 get_transform_matrix() const;
		/// @brief Gets the currently computed translation.
		glm::vec4 get_transform_translation() const;
		/// @brief Returns the computed scaling parameter if requested.
		double get_transform_scaling() const;

		/// @brief Gets the source distribution data.
		point_tensor_t get_source_point_cloud_py() const override;
		/// @brief Gets the target distribution data.
		point_tensor_t get_target_point_cloud_py() const override;

		/// @brief Returns the size of the source distribution. Used for information in Python's ``__repr__`` function.
		std::uint32_t get_source_distribution_size() const override;
		/// @brief Returns the size of the target distribution. Used for information in Python's ``__repr__`` function.
		std::uint32_t get_target_distribution_size() const override;

	private:
		/// @brief Loads the model, and transforms the "target" with the known transform.
		void intialize_and_transform_models();

	protected:
		const std::string source_file_path;
		const std::string target_file_path;

		glm::mat4 computed_transform;
		glm::vec4 computed_translation;
		double computed_scaling;

		Model source_model;
		Model target_model;
	};

}// namespace spot_wrappers

/// @brief Declares a GLM matrix type that can then be used within a Python module defined using pybind11.
/// @tparam C The number of columns of the matrix.
/// @tparam R The number of rows of the matrix.
/// @tparam T The internal data type of the matrix.
/// @tparam Q The GLM qualifier of the matrix.
/// @param matrix_ctad A matrix used to automatically gather the right template arguments. Unused otherwise.
/// @param type_name A string representing the matrix type from within Python.
/// @param m The pybind11 module to bind the type to.
/// @returns A ``pybind11::class_<>`` template containing the required buffer information.
template <glm::length_t C, glm::length_t R, typename T, glm::qualifier Q>
auto define_glm_type_matrix(glm::mat<C, R, T, Q> const& matrix_ctad, const std::string& type_name, pybind11::module_& m) {
	return pybind11::class_<glm::mat<C, R, T, Q>>(m, type_name.c_str(), pybind11::buffer_protocol())
		.def_buffer([](glm::mat<C, R, T, Q>& matrix) -> pybind11::buffer_info {
			return pybind11::buffer_info(
				&matrix[0][0],								/* Pointer to buffer */
				sizeof(T), 									/* Size of one scalar */
				pybind11::format_descriptor<T>::format(),	/* Python struct-style format descriptor */
				2, 											/* Number of dimensions */
				{ R, C }, 									/* Buffer dimensions */
				{ sizeof(T) * R, sizeof(T) } 				/* Strides in bytes for each index */
			);
		});
}

/// @brief Declares a GLM vector type that can then be used within a Python module defined using pybind11.
/// @tparam L The length of the vector.
/// @tparam T The internal data type of the vector.
/// @tparam Q The GLM qualifier of the vector.
/// @param vec_ctad A vector used to automatically gather the right template arguments. Unused otherwise.
/// @param type_name A string representing the vector type from within Python.
/// @param m The pybind11 module to bind the type to.
/// @returns A ``pybind11::class_<>`` template containing the required buffer information.
template <glm::length_t L, typename T, glm::qualifier Q>
auto define_glm_type_vector(glm::vec<L, T, Q> const& vec_ctad, const std::string& type_name, pybind11::module_& m) {
	return pybind11::class_<glm::vec<L, T, Q>>(m, type_name.c_str(), pybind11::buffer_protocol())
		.def_buffer([](glm::vec<L, T, Q>& vector) -> pybind11::buffer_info {
			return pybind11::buffer_info(
				&vector[0],									/* Pointer to buffer */
				sizeof(T), 									/* Size of one scalar */
				pybind11::format_descriptor<T>::format(),	/* Python struct-style format descriptor */
				1, 											/* Number of dimensions */
				{ L },	 									/* Buffer dimensions */
				{ sizeof(T) } 								/* Strides in bytes for each index */
			);
		});
}

#endif //SPOT__SPOT_WRAPPERS_HPP_
