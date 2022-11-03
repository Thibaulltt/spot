//
// Created by thibault on 21/09/22.
//

#include "../../src/spot_wrappers.hpp"
#include "../path_setup.hpp"

int main(int argc, char* argv[]) {

	spot_wrappers::FISTWrapperRandomModels random_points(700, 1000, 1.0);
	spot_wrappers::FISTWrapperSameModel same_model(get_path_to_test_files("Datasets/models/bunny.off"));
	spot_wrappers::FISTWrapperDifferentModels different_models(
		get_path_to_test_files("Datasets/models/bunny.off"),
		get_path_to_test_files("Datasets/models/triceratops.off")
	);

	return EXIT_SUCCESS;
}