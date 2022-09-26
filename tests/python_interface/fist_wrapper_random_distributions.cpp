//
// Created by thibault on 22/09/22.
//

#include "../../spot_wrappers.hpp"
#include "../path_setup.hpp"

int main(int arc, char* argv[]) {

	spot_wrappers::FISTWrapperRandomModels random_models(700, 1000, 1.0);
	random_models.set_maximum_iterations(200);
	random_models.set_maximum_directions(100);

	random_models.compute_transformation(true);

	fmt::print("Final transformations :\n{}\nFinal translation :\n{}\nFinal scaling :{}\n",
		random_models.get_computed_matrix(),
		random_models.get_computed_translation(),
		random_models.get_computed_scaling()
	);

	return EXIT_SUCCESS;
}