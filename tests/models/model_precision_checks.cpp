//
// Created by thibault on 05/10/22.
// This test aims to answer a simple question : is the Point<> type really precise ?
// I've got some weird issues with precision in other parts of the code, and it doesn't make sense why the precision of
// a float (fp32) value should have a ULP precision of 2e-5 when it is supposed to be 1e-7.
//

#include <cstring> // needed for memset() ...
#include <cmath>
#include <array>
#include <random>

#include "../path_setup.hpp"
#include "../../model.hpp"
#include "../../glm_bridge.hpp"
#include "../../fmt_bridge.hpp"

/// @brief Generate a sample of a given dimension scaled up by a given factor.
/// @tparam DIM The dimension of the sample to generate.
/// @tparam T The data type of the sample to generate.
/// @param scale_factor The scale factor to apply to all points generated.
/// @param generator The random number generator passed to the function, which can be called via operator() and returns a T type.
/// @returns A sample of in a 'DIM'-dimensional space.
template <int DIM, typename T>
Point<DIM, T> generate_random_point(const T& scale_factor, const std::function<T()>& generator) {
	Point<DIM, T> generated{};

	for (int i = 0; i < DIM; ++i) {
		generated[i] = generator() * scale_factor;
	}

	return generated;
}

/// @brief Generate a vector of a given size containing points in the unit cube, scaled up by a given factor.
/// @tparam DIM The dimension of the samples to generate.
/// @tparam T The data type of the samples to generate.
/// @param size_to_generate The number of samples to generate.
/// @param scale_factor The scale factor to apply to all points generated.
/// @param generator The random number generator passed to the function, which can be called via operator() and returns a T type.
/// @returns A vector of 'size_to_generate' samples in a 'DIM'-dimensional space.
template <int DIM, typename T>
std::vector<Point<DIM, T>> generate_random_point_vector(const std::size_t size_to_generate, const T& scale_factor, const std::function<T()>& generator) {
	std::vector<Point<DIM, T>> generated_points(size_to_generate);

	for (auto& point : generated_points) {
		for (int i = 0; i < DIM; ++i) {
			point[i] = generator() * scale_factor;
		}
	}

	return generated_points;
}

/// @brief Adds a given translation to an array of point coordinates in n-D
/// @tparam DIM The sample space's dimensions
/// @tparam T The sample space's data type.
/// @param original_values The vector to modify in-place.
/// @param translation The translation vector to apply
template <int DIM, typename T>
void add_translation_vector_inplace(std::vector<Point<DIM, T>>& original_values, const Point<DIM, T> translation) {
	for (auto& coordinates : original_values) {
		coordinates += translation;
	}
}

/// @brief Checks the two given samples are 'close', i.e. equal up to an epsilon value.
/// @tparam DIM The sample space's dimensions
/// @tparam T The sample space's data type.
/// @returns True if the samples are close, false if they aren't.
template <int DIM, typename T>
bool are_epsilon_close(const Point<DIM, T>& lhs, const Point<DIM, T>& rhs, const T epsilon) {
	bool is_all_under_epsilon = true;
	for (int i = 0; i < DIM; ++i) {
		is_all_under_epsilon &= (std::abs(rhs[i] - lhs[i]) < epsilon);
	}
	return is_all_under_epsilon;
}

/// @brief Returns all comparison results for vectors of point coordinates.
/// @tparam DIM The sample space's dimensions
/// @tparam T The sample space's data type.
/// @returns A vector of two possible values : true if the samples were close and false if they weren't for all points.
template <int DIM, typename T>
std::vector<bool> are_epsilon_close(const std::vector<Point<DIM, T>>& lhs, const std::vector<Point<DIM, T>>& rhs, const T epsilon) {
	using index_t = typename std::vector<Point<DIM, T>>::size_type;
	std::vector<bool> results(lhs.size());
	index_t index = 0;

	for (; index < lhs.size(); index++) {
		results[index] = are_epsilon_close(lhs[index], rhs[index], epsilon);
	}

	return results;
}

/// @brief Computes how many samples were close up to an epsilon value.
auto count_all_close_samples(const std::vector<bool>& closeness) -> std::decay_t<decltype(closeness)>::size_type {
	using count_t = std::decay_t<decltype(closeness)>::size_type;

	count_t all_close = 0;
	for (const auto are_close : closeness) {
		all_close += static_cast<count_t>(are_close);
	}

	return all_close;
}

template <int DIM, typename T>
bool is_Point_type_really_precise(const std::size_t sample_count, const T& scale_factor, const T& maximum_epsilon_acceptable) {
	using point_t = Point<DIM, T>;

	std::default_random_engine random_engine{};
	std::uniform_real_distribution<T> distribution_generator(T(0), T(1) * scale_factor);
	auto generation_functor = [&distribution_generator, &random_engine]() -> T {
		return distribution_generator(random_engine);
	};

	// Generate points within the unit cube :
	std::vector<point_t> points_original = generate_random_point_vector<DIM, T>(sample_count, scale_factor, generation_functor);
	for (auto& point : points_original) {
		for(int i = 0; i < DIM; ++i) {
			point[i] = distribution_generator(random_engine);
		}
	}

	// Generate a random vector :
	point_t translation_vector = generate_random_point<DIM, T>(scale_factor, generation_functor);
	// Copy original into the 'shifted' points array :
	std::vector<point_t> points_shifted(points_original);
	// Apply translation :
	add_translation_vector_inplace(points_shifted, translation_vector);
	// Substract translation :
	add_translation_vector_inplace(points_shifted, -translation_vector);
	// Check both samples are somewhat close :
	auto closeness = are_epsilon_close(points_original, points_shifted, maximum_epsilon_acceptable);
	auto count = count_all_close_samples(closeness);

	fmt::print(
		"Samples close : {: >5d}/{: >5d} up to an epsilon precision of {: >12.10e}\n",
		count, sample_count, maximum_epsilon_acceptable
	);

	return count == sample_count;
}

bool check_epsilons(float epsilon_flt, double epsilon_dbl) {
	auto is_close_float = is_Point_type_really_precise<3, float>(10000, 15.0f, epsilon_flt);
	auto is_close_double = is_Point_type_really_precise<3, double>(10000, double(15.0), epsilon_dbl);

	fmtdbg("is_close_float ({: >24.22e}) : {:s}", epsilon_flt, is_close_float);
	fmtdbg("is_close_double({: >24.22e}) : {:s}", epsilon_dbl, is_close_double);

	return is_close_float && is_close_double;
}

std::pair<float, double> find_acceptable_epsilons_for_Point_type() {
	constexpr std::array<float, 9> epsilons_flt = {1e-1f, 1e-2f, 1e-3f, 1e-4f, 1e-5f, 1e-6f, 1e-7f, 1e-8f, 1e-9f};
	constexpr std::array<double, 9> epsilons_dbl = {1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9};

	float maximum_epsilon_acceptable_flt = 1.0f;
	for (const auto epsilon_flt : epsilons_flt) {
		if (check_epsilons(epsilon_flt, 1.0)) {
			maximum_epsilon_acceptable_flt = epsilon_flt;
		} else {
			break;
		}
	}
	double maximum_epsilon_acceptable_dbl = 1.0;
	for (const auto epsilon_dbl : epsilons_dbl) {
		if (check_epsilons(1.0f, epsilon_dbl)) {
			maximum_epsilon_acceptable_dbl = epsilon_dbl;
		} else {
			break;
		}
	}

	fmtdbg("Maximum epsilon for float  : {: >8.6e}", maximum_epsilon_acceptable_flt);
	fmtdbg("Maximum epsilon for double : {: >8.6e}", maximum_epsilon_acceptable_dbl);

	fmtdbg("<limits> : std::numeric_limits<float>::epsilon()  = {: >8.6e}",  std::numeric_limits<float>::epsilon());
	fmtdbg("<limits> : std::numeric_limits<double>::epsilon() = {: >8.6e}", std::numeric_limits<double>::epsilon());

	return {maximum_epsilon_acceptable_flt, maximum_epsilon_acceptable_dbl};
}

int main(int argc, char* argv[]) {
	float eps_flt;
	double eps_dbl;
	std::tie(eps_flt, eps_dbl) = find_acceptable_epsilons_for_Point_type();
	//return is_close_float && is_close_double ? EXIT_SUCCESS : EXIT_FAILURE;
	return EXIT_SUCCESS;
}
