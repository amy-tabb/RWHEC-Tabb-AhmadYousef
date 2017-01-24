/*
 * RWHEC_Nov2015_main.hpp
 *
 *  Created on: Nov 16, 2015
 *      Author: atabb
 */

#ifndef TABB_AHMADYOUSEF_RWHEC_JAN2017_MAIN_HPP_
#define TABB_AHMADYOUSEF_RWHEC_JAN2017_MAIN_HPP_

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Calibration2.hpp"
#include <iostream>
#include "CostFunctions.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;


int RobotWorldHandEyeCalibration(double square_mm_height, double square_mm_width,
		int chess_h, int chess_w, string source_dir, string write_dir,  bool do_camcali, bool do_rwhec, bool do_reconstruction, bool verbose);

void WriteCaliFile(CaliObjectOpenCV2* CO, std::ofstream& out);

void WriteCaliFile(CaliObjectOpenCV2* CO, vector<Matrix>& As, vector<Matrix>& Bs, Matrix& X, Matrix& Z, std::ofstream& out);

void ReadRobotFileDensoArm(string write_dir, vector<Matrix>& Bs);

void ReadRobotFileRobotCaliTxt(string filename, vector<Matrix>& Bs);


double AssessErrorWhole(vector<Matrix>& As, vector<Matrix>& Bs, Matrix& X, Matrix& Z);

double AssessRotationError(vector<Matrix>& As, vector<Matrix>& Bs, Matrix& X, Matrix& Z);

double AssessRotationErrorAxisAngle(vector<Matrix>& As, vector<Matrix>& Bs, Matrix& X, Matrix& Z);

double AssessTranslationErrorDenominator(vector<Matrix>& As, vector<Matrix>& Bs, Matrix& X, Matrix& Z);

double AssessTranslationError(vector<Matrix>& As, vector<Matrix>& Bs, Matrix& X, Matrix& Z);

double CalculateReprojectionError(CaliObjectOpenCV2* CO, vector<Matrix>& As, vector<Matrix>& Bs,
			Matrix& X, Matrix& Z, std::ofstream& out, string directory, int cam_number);

void WritePatterns(double* pattern_points, int chess_h, int chess_w, int index_number, string outfile);

#endif /* TABB_AHMADYOUSEF_RWHEC_JAN2017_MAIN_HPP_ */
