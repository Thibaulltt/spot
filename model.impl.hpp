#include <fstream>
#include <fmt/core.h>
#include <fmt/color.h>
#include <iostream>
#include <algorithm>
#include <functional>

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
	throw std::logic_error("Cannot construct empty model.");
}

Model::Model(const std::vector<glm::vec3>& vertices, const std::vector<glm::uvec3> _triangles) :
	positions(vertices.cbegin(), vertices.cend()), triangles(_triangles) {}
	// Note : range-based ctor of vector is supposed to perform the conversions automatically if an explicit cast op exists.

Model::Model(const Model& _other) = default;

void Model::apply_transform(const glm::mat3 matrix) {
	std::for_each(positions.begin(), positions.end(), [&](Point<3, float>& position) {
		// Dirty : casts to glm::vec3, compute and re-cast into Point<>
		glm::vec3 glm_pos{position[0], position[1], position[2]};
		position = Point<3, float>(glm_pos * matrix);
	});
}

void Model::apply_translation(const glm::vec3 translate) {
	Point<3, float> t{translate};
	std::for_each(this->positions.begin(), this->positions.end(),
		[t](Point<3, float> const& p) {
			return p + t;
		}
	);
}

void Model::apply_scaling(double scaling = 1.0, bool center_before_scaling = true) {
	Point<3, float> center{};
	if (center_before_scaling) {
		std::for_each(this->positions.begin(), this->positions.end(),
			[&center](Point<3, float> const& p) {
				center += p;
			}
		);
		this->apply_translation(glm::to_vec(-center));
	}

	std::for_each(this->positions.begin(), this->positions.end(),
		[scaling](Point<3, float>& p){
			p *= static_cast<float>(scaling);
		}
	);

	if (center_before_scaling) {
		this->apply_translation(glm::to_vec(center));
	}
}
