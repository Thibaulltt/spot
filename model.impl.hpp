//
// Created by thib on 24/08/22.
//
#include <fstream>
#include <fmt/core.h>
#include <fmt/color.h>
#include <iostream>
#include <algorithm>
#include <functional>

const aiScene* load_model_file(const std::string load_path) {
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(load_path.c_str(),
		aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
		aiProcess_CalcTangentSpace
	);

	if (scene == nullptr) {
		std::cout << "Error : could not load the scene/model with assimp.\nError message :\n\t- " << importer.GetErrorString() << '\n';
		throw std::runtime_error("Coud not load the file.");
	}
	return scene;
}

void print_aiScene_contents(const aiScene* scene) {
	/* Debugging purposes only. */

	fmt::print("~~~ Printing contents of aiScene {} ~~~\n", fmt::ptr(scene));
	fmt::print("- Name of the scene : {}\n", scene->mName.C_Str());
	fmt::print("- Animations  : ");
	if (scene->mNumAnimations == 0) {
		fmt::print(fmt::fg(fmt::color::red), "<none>\n");
	} else {
		fmt::print(fmt::emphasis::italic | fmt::fg(fmt::color::blue), "{} animations\n", scene->mNumAnimations);
	}
	fmt::print("- Cameras     : ");
	if (scene->mNumCameras == 0) {
		fmt::print(fmt::fg(fmt::color::red), "<none>\n");
	} else {
		fmt::print(fmt::emphasis::italic | fmt::fg(fmt::color::blue), "{} cameras\n", scene->mNumCameras);
		// lights materials meshes texture
	}
	fmt::print("- Lights      : ");
	if (scene->mNumLights == 0) {
		fmt::print(fmt::fg(fmt::color::red), "<none>\n");
	} else {
		fmt::print(fmt::emphasis::italic | fmt::fg(fmt::color::blue), "{} lights\n", scene->mNumLights);
	}
	fmt::print("- Materials   : ");
	if (scene->mNumMaterials == 0) {
		fmt::print(fmt::fg(fmt::color::red), "<none>\n");
	} else {
		fmt::print(fmt::emphasis::italic | fmt::fg(fmt::color::blue), "{} materials\n", scene->mNumMaterials);
	}
	fmt::print("- Meshes      : ");
	if (scene->mNumMeshes == 0) {
		fmt::print(fmt::fg(fmt::color::red), "<none>\n");
	} else {
		fmt::print(fmt::emphasis::italic | fmt::fg(fmt::color::blue), "{} meshes\n", scene->mNumMeshes);
	}
	fmt::print("- Textures    : ");
	if (scene->mNumTextures == 0) {
		fmt::print(fmt::fg(fmt::color::red), "<none>\n");
	} else {
		fmt::print(fmt::emphasis::italic | fmt::fg(fmt::color::blue), "{} textures\n", scene->mNumTextures);
	}
}

Model load_off_file(const std::string& filename)
{
	std::cout << "Opening " << filename << std::endl;
	std::vector<glm::vec3> vertices;
	std::vector<glm::uvec3> triangles;

	// open the file
	std::ifstream myfile;
	myfile.open(filename.c_str());
	if (!myfile.is_open())
	{
		std::cout << filename << " cannot be opened" << std::endl;
		return {};
	}

	std::string magic_s;

	myfile >> magic_s;

	// check if it's OFF
	if( magic_s != "OFF" )
	{
		std::cout << magic_s << " != OFF :   We handle ONLY *.off files." << std::endl;
		myfile.close();
		return {};
	}

	int n_vertices , n_faces , dummy_int;
	myfile >> n_vertices >> n_faces >> dummy_int;

	// Clear any vertices
	vertices.clear();

	// Read the vertices
	for( int v = 0 ; v < n_vertices ; ++v )
	{
		float x , y , z;
		myfile >> x >> y >> z ;
		vertices.emplace_back(glm::vec3( x , y , z ));
	}

	// Clear any triangles
	triangles.clear();

	// Read the triangles
	for( int f = 0 ; f < n_faces ; ++f )
	{
		int n_vertices_on_face;
		myfile >> n_vertices_on_face;
		if( n_vertices_on_face == 3 )
		{
			unsigned int _v1 , _v2 , _v3;
			myfile >> _v1 >> _v2 >> _v3;
			triangles.emplace_back(glm::uvec3(_v1, _v2, _v3));
		}
		else if( n_vertices_on_face == 4 )
		{
			unsigned int _v1 , _v2 , _v3 , _v4;

			myfile >> _v1 >> _v2 >> _v3 >> _v4;
			triangles.emplace_back(glm::uvec3(_v1, _v2, _v3));
			triangles.emplace_back(glm::uvec3(_v1, _v3, _v4));
		}
		else
		{
			std::cout << "We handle ONLY *.off files with 3 or 4 vertices per face" << std::endl;
			myfile.close();
			return {};
		}
	}
	return {vertices, triangles}; // Calls 'Model(v, t)'
}

Model::Model() : positions(), triangles() {
	throw std::logic_error("Cannot contruct empty model.");
}

Model::Model(const std::vector<glm::vec3>& vertices, const std::vector<glm::uvec3> _triangles) :
	positions(vertices.cbegin(), vertices.cend()), triangles(_triangles) {}
	// Note : range-based ctor of vector is supposed to perform the conversions automatically if an explicit cast op exists.

Model::Model(const Model& _other) : positions(_other.positions), triangles(_other.triangles) {}

void Model::apply_transform(const glm::mat3 matrix) {
	std::for_each(positions.begin(), positions.end(), [&](Point<3, float>& position) {
		// Dirty : casts to glm::vec3, compute and re-cast into Point<>
		glm::vec3 glm_pos{position[0], position[1], position[2]};
		position = Point<3, float>(glm_pos * matrix);
	});
}

void Model::apply_translation(const glm::vec3 translate) {
	Point<3, float> t{translate};
	std::for_each(this->positions.begin(), this->positions.end(), std::bind(&Point<3,float>::operator+=, std::placeholders::_1, t));
}
