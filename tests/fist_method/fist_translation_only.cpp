//
// Created by thib on 26/08/22.
//

#include "../../UnbalancedSliced.h"
#include "../../model.hpp"

#include <glm/gtx/io.hpp>

int main(int argc, char* argv[]) {
	omp_set_nested(0);

	int FIST_iters = 200;
	int slices = 100;
	UnbalancedSliced sliced;

	// Load models :
	auto model_reference = load_off_file("/home/thib/Documents/data/medmax/meshes/Test/bunny.off");
	auto model_translated = Model(model_reference);
	glm::vec3 random_translation = glm::random(glm::vec3{});
	std::cout << "Applying translation on " << model_reference.positions.size() << " vertices  : " << random_translation << '\n';
	// We apply negative the offset, because we want the program to compute the original vector above :
	model_translated.apply_translation(-random_translation);

	std::vector<double> rot(9);
	std::vector<double> trans(3);
	double scaling;
	sliced.fast_iterative_sliced_transport(FIST_iters, slices, model_translated.positions, model_reference.positions, rot, trans, true, scaling);

	std::cout << "Scale: " << scaling << std::endl;
	fmt::print("[                      Rotation                      ] [    Translation   ]\n\n");
	fmt::print("[ {: >16.10f} {: >16.10f} {: >16.10f} ] [ {: >16.10f} ]\n", rot[0], rot[1], rot[2], trans[0]);
	fmt::print("[ {: >16.10f} {: >16.10f} {: >16.10f} ] [ {: >16.10f} ]\n", rot[3], rot[4], rot[5], trans[1]);
	fmt::print("[ {: >16.10f} {: >16.10f} {: >16.10f} ] [ {: >16.10f} ]\n", rot[6], rot[7], rot[8], trans[2]);
	std::cout << std::endl;

	return 0;
}