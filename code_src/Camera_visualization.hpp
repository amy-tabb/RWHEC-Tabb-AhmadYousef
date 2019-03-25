/*
 * camera_visualization.hpp
 *
 *  Created on: Mar 25, 2019
 *      Author: atabb
 */

#ifndef CAMERA_VISUALIZATION_HPP_
#define CAMERA_VISUALIZATION_HPP_

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


using namespace Eigen;

#include <list>
#include <iostream>



#include <iostream>


#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <list>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <inttypes.h>

using namespace std;

int create_camera(Matrix3d& internal, Matrix4d& external, Vector3d& C, int r, int g, int b, int rows, int cols, string ply_file);

int create_camera(Matrix3d& internal, Matrix4d& external, int r, int g, int b, int rows, int cols, string ply_file);

#endif /* CAMERA_VISUALIZATION_HPP_ */
