/*
 * RWHEC_Nov2015_main.hpp
 *
 *  Created on: Nov 16, 2015
 *      Author: atabb
 */

#ifndef TABB_AHMADYOUSEF_RWHEC_MAY2018_MAIN_HPP_
#define TABB_AHMADYOUSEF_RWHEC_MAY2018_MAIN_HPP_

#include "ceres/ceres.h"
#include "glog/logging.h"
#include "Calibration2.hpp"
#include <iostream>
#include "CostFunctions.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


using namespace Eigen;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;


int RobotWorldHandEyeCalibration(double square_mm_height, double square_mm_width,
		int chess_h, int chess_w, string source_dir, string write_dir,  bool do_camcali, bool do_rwhec, bool do_reconstruction, bool verbose);

void WriteCaliFile(CaliObjectOpenCV2* CO, std::ofstream& out);

void WriteCaliFile(CaliObjectOpenCV2* CO, vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z, std::ofstream& out);
//void WriteCaliFile(CaliObjectOpenCV2* CO, vector<MatrixXd>& As, vector<MatrixXd>& Bs, Matrix& X, Matrix& Z, std::ofstream& out);

//void ReadRobotFileDensoArm(string write_dir, vector<Matrix4d>& Bs);

void ReadRobotFileRobotCaliTxt(string filename, vector<Matrix4d>& Bs);


double AssessErrorWhole(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z);

double AssessRotationError(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z);

double AssessRotationErrorAxisAngle(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z);

double AssessTranslationErrorDenominator(vector<Matrix4d>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z);

double AssessTranslationError(vector<MatrixXd>& As, vector<Matrix4d>& Bs, Matrix4d& X, Matrix4d& Z);

double CalculateReprojectionError(CaliObjectOpenCV2* CO, vector<MatrixXd>& As, vector<Matrix4d>& Bs,
			Matrix4d& X, Matrix4d& Z, std::ofstream& out, string directory, int cam_number);

void WritePatterns(double* pattern_points, int chess_h, int chess_w, int index_number, string outfile);

string FindValueOfFieldInFile(string filename, string fieldTag, bool seperator);

void EnsureDirHasTrailingBackslash(string& write_directory);

#endif /* TABB_AHMADYOUSEF_RWHEC_MAY2018_MAIN_HPP_ */
