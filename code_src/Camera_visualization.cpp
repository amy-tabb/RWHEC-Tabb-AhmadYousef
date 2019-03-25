/*
 * camera_visualization.cpp
 *
 *  Created on: Mar 25, 2019
 *      Author: atabb
 */


#include "Camera_visualization.hpp"


typedef uint64_t int_type_t;


void camera(Matrix3d& Kinv, float max_u, float max_v, float mag, vector< Vector3d >& vertex_coordinates ) {

	Vector3d a;
	Vector3d b;
	Vector3d c;
	Vector3d d;

	//cout << "max u , max v, mag " << max_u << ", " << max_v << ", " << mag << endl;
	//cout << "Kinv " << endl << Kinv << endl;
	//
	//
	//	Vector4d C;
	//	C << 0, 0, 0, 1;

	Vector3d x;
	x << 0, 0, 1;

	a = mag*Kinv*x;

	//cout << "a, " << endl << a << endl;

	x << max_u, 0, 1;
	b = mag*Kinv*x;

	x << max_u, max_v, 1;
	c = mag*Kinv*x;

	x << 0, max_v, 1;
	d = mag*Kinv*x;

	vertex_coordinates.push_back(a);
	vertex_coordinates.push_back(d);
	vertex_coordinates.push_back(c);
	vertex_coordinates.push_back(b);



}


int create_camera(Matrix3d& internal, Matrix4d& external, Vector3d& C, int r, int g, int b, int rows, int cols, string ply_file)
{
	vector< Vector3d > vertex_coordinates;
	vector< Vector3d > vertex_normals;
	vector< vector<int> > face_indices;
	vector< vector<int> > edge_indices;
	vector<Vector3d> colors;

	//vector< Vector3d >& vertex_coordinates_external;
	//////////////////////////roi box///////////////////////////////////

	Matrix3d R;
	//Vector3d C;
	Vector3d t;

	//t = -RC
	// C = -R.t*t.
	/// deal with some problems ...

	//external.setIdentity();


	Matrix3d Kinv = internal.inverse();

	//Kinv = internal; //.inverse();
	//cout << "Kinv " << endl << Kinv << endl;

	//	MatrixXd temp =  internal * Kinv;
	//
	//	cout << temp << endl;

	for (int r = 0; r < 3; r++){
		for (int c = 0; c < 3; c++){
			R(r, c) = external(r, c);
		}
		t(r) = external(r, 3);
	}

	//C = -R.transpose()*t;
	//C = R*t;
	//cout << "C " << C << endl;

	Matrix3d Rinv = R.transpose();

	//R = Rtemp;
	cout << "camera VIS line 168 cols, rows  " << cols << ", " << rows << endl; //current_position[0] << ", " << current_position[1] << ", " << current_position[2]

	camera(Kinv, cols, rows, 40.0, vertex_coordinates);
	//	if (OpenGL){
	//		camera_OpenGL(Kinv, cols, rows, 40.0, vertex_coordinates);
	//	}	else {
	//
	//	}


	Vector3d tempV;

	/// R is Rt
	//vertices[i] = R*vertices[i] + C;

	for (int i = 0; i < 4; i++){
		tempV = vertex_coordinates[i];

		//		if (OpenGL){
		//			/// z component needs to be switched ....
		//			tempV(2) *= -1;
		//
		//		}

		//vertex_coordinates[i] = Rinv*(tempV - t);;

		vertex_coordinates[i] = Rinv*(vertex_coordinates[i] - t);

	}

	Vector3d diff_vector;
	//	if (OpenGL){
	//		for (int i = 0; i < 4; i++){
	//			diff_vector = C - vertex_coordinates[i];
	//			vertex_coordinates[i] = C + diff_vector;
	//		}
	//		//		/// need to move vertices around ... so that the first one is the left corner.  In the flip, will get this out of order.
	//		//
	//	}



	vertex_coordinates.push_back(C);


	Vector3d cp;

	// do normals after this is working ...
	//cp =

	int vertex_number = 0;

	//cout << "Line 220 " << endl;

	// front face.
	vector<int> face;
	face.push_back(vertex_number);
	face.push_back(vertex_number + 3);
	face.push_back(vertex_number + 1);

	face_indices.push_back(face);


	face.clear();

	face.push_back(vertex_number + 2);
	face.push_back(vertex_number + 1);
	face.push_back(vertex_number + 3);

	face_indices.push_back(face);


	face.clear();
	// a side.

	face.push_back(vertex_number);
	face.push_back(vertex_number + 4);
	face.push_back(vertex_number + 3);


	face_indices.push_back(face);

	face.clear();

	face.push_back(vertex_number);
	face.push_back(vertex_number + 1);
	face.push_back(vertex_number + 4);


	face_indices.push_back(face);

	face.clear();

	face.push_back(vertex_number + 1);
	face.push_back(vertex_number + 2);
	face.push_back(vertex_number + 4);


	face_indices.push_back(face);

	face.clear();

	face.push_back(vertex_number + 2);
	face.push_back(vertex_number + 3);
	face.push_back(vertex_number + 4);


	face_indices.push_back(face);

	face.clear();


	vertex_number += 5;


	std::ofstream out;
	out.open(ply_file.c_str());


	out << "ply" << endl;
	out << "format ascii 1.0" << endl;
	out << "element vertex " << vertex_coordinates.size() << endl;
	out << "property float x" << endl;
	out << "property float y" << endl;
	out << "property float z" << endl;
	//out << "property float nx" << endl;
	//out << "property float ny" << endl;
	//out << "property float nz" << endl;
	out << "property uchar red" << endl;
	out << "property uchar green" << endl;
	out << "property uchar blue" << endl;
	out << "property uchar alpha" << endl;
	out << "element face " << face_indices.size() << endl;
	out << "property list uchar int vertex_indices"<< endl;

	out << "end_header" << endl;

	unsigned int zero = 0;
	for (int i = 0, nc = vertex_coordinates.size(); i < nc; i++){
		out << vertex_coordinates[i](0) << " " << vertex_coordinates[i](1) << " " << vertex_coordinates[i](2) << " ";

		if (i == 0){
			out << " 255 255 255 255" << endl;
		}	else {
			out << r << " " << g << " " << b << " 255 " << endl;
		}


	}

	for (int i = 0; i < int(face_indices.size()); i++){

		out << face_indices[i].size() << " ";


		for (int j = 0; j < int(face_indices[i].size()); j++){
			out << face_indices[i].at(j) << " ";
		}


		out << endl;

	}


	out.close();



	return(0);



}

int create_camera(Matrix3d& internal, Matrix4d& external, int r, int g, int b, int rows, int cols, string ply_file){

	//cout << "Internal " << internal << endl;
	//cout << "external " << external << endl;
	Vector3d C;
	Vector3d t;
	Matrix3d R;
	// t = -RC, so C = -inv(R)*t
	for (int r0 = 0; r0 < 3; r0++){
		for (int c = 0; c < 3; c++){
			R(r0, c) = external(r0, c);
		}

		t(r0) = external(r0, 3);
	}

	//cout << "R " << R << endl;
	//cout << "T " << t << endl;
	C = -R.inverse()*t;

	//cout << "C " << endl << C << endl;

	create_camera(internal, external, C, r, g, b, rows, cols, ply_file);

	return 0;
}
