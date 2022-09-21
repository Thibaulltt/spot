//
// Created by thibault on 21/09/22.
//

#include "../../spot_wrappers.hpp"

int main(int argc, char* argv[]) {

	spot_wrappers::FISTWrapperRandomModels random_points(700, 1000, 1.0);
	spot_wrappers::FISTWrapperSameModel same_model("/home/thibault/Documents/miccai/durand_export/meshmand.off");
	spot_wrappers::FISTWrapperDifferentModels different_models(
		"/home/thibault/Documents/miccai/durand_export/meshmand.off",
		"/home/thibault/Documents/miccai/durand_export/fib.off"
	);

	return EXIT_SUCCESS;
}