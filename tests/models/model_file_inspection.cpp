//
// Created by thib on 25/08/22.
//

#include "../../model.hpp"

int main(int argc, char* argv[]) {

	const aiScene* scene = load_model_file("/home/thib/Documents/data/medmax/meshes/Test/bunny.off");
	print_aiScene_contents(scene);

	return EXIT_SUCCESS;
}