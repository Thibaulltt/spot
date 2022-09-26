//
// Created by thib on 26/08/22.
// Tests out the FIST algorithm on real datasets, with a translation only.
//

#include "../../UnbalancedSliced.h"
#include "../../model.hpp"
#include "../path_setup.hpp"

#include <glm/gtx/io.hpp>

int main() {
	omp_set_nested(0);

	int FIST_iters = 200;
	int slices = 100;
	UnbalancedSliced sliced;

	// Load models :
	auto model_reference = load_off_file(get_path_to_test_files("Datasets/models/bunny.off"));
	auto model_translated = Model(model_reference);
	glm::vec3 random_translation = glm::sphericalRand(1.0f);
	std::cout << "Applying translation on " << model_reference.positions.size() << " vertices  : " << random_translation << '\n';
	// We apply negative the offset, because we want the program to compute the original vector above :
	model_translated.apply_translation(-random_translation);

	std::vector<double> rot(9);
	std::vector<double> trans(3);
	double scaling;
	auto logger = std::make_unique<micro_benchmarks::TimingsLogger>(FIST_iters);
	logger = sliced.fast_iterative_sliced_transport(
		FIST_iters, slices, model_translated.positions, model_reference.positions, rot, trans, true, scaling, std::move(logger));
	logger->print_timings("From CTest executable test_fist_translation_only", "[Results]");

	std::cout << "Scale: " << scaling << std::endl;
	fmt::print("[                                  Rotation                                  ] [        Translation       ]\n");
	fmt::print("[ {: >+24.10f} {: >+24.10e} {: >+24.10e} ] [ {: >+24.10f} ]\n", rot[0], rot[1], rot[2], trans[0]);
	fmt::print("[ {: >+24.10e} {: >+24.10f} {: >+24.10e} ] [ {: >+24.10f} ]\n", rot[3], rot[4], rot[5], trans[1]);
	fmt::print("[ {: >+24.10e} {: >+24.10e} {: >+24.10f} ] [ {: >+24.10f} ]\n", rot[6], rot[7], rot[8], trans[2]);
	std::cout << std::endl;
	glm::vec3 glm_trans = glm::vec3(glm::dvec3(trans[0], trans[1], trans[2]));
	// Check equality :
	glm::bvec3 near_epsilon = glm::epsilonEqual(random_translation, glm_trans, 1e-6f);

	std::cout << "Equality ? " << near_epsilon << '\n';

	return glm::all(near_epsilon) ? EXIT_SUCCESS : EXIT_FAILURE;
}