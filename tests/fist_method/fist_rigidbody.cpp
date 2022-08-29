//
// Created by thib on 26/08/22.
// Tests out the FIST algorithm on real datasets, with a rigidbody transform.
//

#include "../../UnbalancedSliced.h" // FIXME : If we include this header below model.hpp, causes ADL/named argument lookup failure on memset()
#include "../../model.hpp"

int main() {
	omp_set_nested(0);

	int FIST_iters = 3000;
	int slices = 100;
	UnbalancedSliced sliced;

	// Load models :
	auto model_reference = load_off_file("/home/thib/Documents/data/medmax/meshes/Test/bunny.off");
	auto model_translated = Model(model_reference);
	glm::vec3 random_translation = glm::sphericalRand(1.0f);
	glm::vec3 random_axis = glm::normalize(glm::sphericalRand(1.0f));
	glm::quat random_quat = glm::angleAxis(glm::radians(30.0f), random_axis);
	glm::mat3 random_matrix = glm::mat3_cast(random_quat);
	glm::mat3 inverted_transform = glm::inverse(random_matrix);

	// We apply the inverse transform, because we want the program to compute the original one defined above :
	std::cout << "Applying rotation around axis " << random_axis << " for 30 degrees ...\n";
	model_translated.apply_transform(inverted_transform);
	std::cout << "Applying translation on " << model_reference.positions.size() << " vertices  : " << random_translation << '\n';
	model_translated.apply_translation(-random_translation);
	fmt::print("Source transform : \n");
	fmt::print("[                                  Rotation                                  ] [        Translation       ]\n");
	fmt::print("[ {: >+24.10f} {: >+24.10e} {: >+24.10e} ] [ {: >+24.10f} ]\n", random_matrix[0][0], random_matrix[0][1], random_matrix[0][2], random_translation[0]);
	fmt::print("[ {: >+24.10e} {: >+24.10f} {: >+24.10e} ] [ {: >+24.10f} ]\n", random_matrix[1][0], random_matrix[1][1], random_matrix[1][2], random_translation[1]);
	fmt::print("[ {: >+24.10e} {: >+24.10e} {: >+24.10f} ] [ {: >+24.10f} ]\n", random_matrix[2][0], random_matrix[2][1], random_matrix[2][2], random_translation[2]);
	fmt::print("\n");

	std::vector<double> rot(9);
	std::vector<double> trans(3);
	double scaling;
	sliced.fast_iterative_sliced_transport(FIST_iters, slices, model_translated.positions, model_reference.positions, rot, trans, true, scaling, true);

	std::cout << "Scale: " << scaling << std::endl;
	fmt::print("[                                  Rotation                                  ] [        Translation       ]\n");
	fmt::print("[ {: >+24.10f} {: >+24.10e} {: >+24.10e} ] [ {: >+24.10f} ]\n", rot[0], rot[1], rot[2], trans[0]);
	fmt::print("[ {: >+24.10e} {: >+24.10f} {: >+24.10e} ] [ {: >+24.10f} ]\n", rot[3], rot[4], rot[5], trans[1]);
	fmt::print("[ {: >+24.10e} {: >+24.10e} {: >+24.10f} ] [ {: >+24.10f} ]\n", rot[6], rot[7], rot[8], trans[2]);
	fmt::print("\n");

	// Let GLM perform float to double conversions :
	glm::mat3 glm_rot = glm::mat3(glm::dmat3(*rot.begin()));
	glm::vec3 glm_trans = glm::vec3(glm::dvec3(trans[0], trans[1], trans[2]));
	// Check equality :
	constexpr float maximum_epsilon = 1e-4f;
	glm::bvec3 near_epsilon = glm::epsilonEqual(random_translation, glm_trans, maximum_epsilon);
	auto near_transform = glm::epsilonEqual(random_matrix, glm_rot, glm::mat3(maximum_epsilon));

	glm::mat3 delta_matrix = glm_rot - random_matrix;
	glm::vec3 delta_vector = glm_trans - random_translation;

	std::cout << "Deltas :" << std::endl;
	fmt::print("[                                  Rotation                                  ] [        Translation       ]\n");
	fmt::print("[ {: >+24.10f} {: >+24.10e} {: >+24.10e} ] [ {: >+24.10f} ]\n", delta_matrix[0][0], delta_matrix[0][1], delta_matrix[0][2], delta_vector[0]);
	fmt::print("[ {: >+24.10e} {: >+24.10f} {: >+24.10e} ] [ {: >+24.10f} ]\n", delta_matrix[1][0], delta_matrix[1][1], delta_matrix[1][2], delta_vector[1]);
	fmt::print("[ {: >+24.10e} {: >+24.10e} {: >+24.10f} ] [ {: >+24.10f} ]\n", delta_matrix[2][0], delta_matrix[2][1], delta_matrix[2][2], delta_vector[2]);
	fmt::print("\n");

	fmt::print("Result of equality checks :\n{} - {}\n", near_epsilon, near_transform);

	return glm::all(near_epsilon) && near_transform ? EXIT_SUCCESS : EXIT_FAILURE;
}
