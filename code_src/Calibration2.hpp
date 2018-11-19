/*
 * Calibration2.hpp
 *
 *  Created on: Jun 25, 2014
 *      Author: atabb
 */

#ifndef CALIBRATION2_HPP_
#define CALIBRATION2_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace Eigen;

#include <math.h>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include "DirectoryFunctions.hpp"
#include "StringFunctions.hpp"

using namespace std;

class CaliObjectOpenCV2{
public:
	double mm_width;
	double mm_height;
	int chess_w;
	int chess_h;
	double mean_reproj_error;
	double mean_ext_reproj_error;

	vector< vector< cv::Point2f> > internal_points;
	vector< vector< cv::Point2f> > external_points;
	vector< vector< cv::Point2f> > all_points;
	vector< vector< cv::Point3f> > all_3d_corners;


	vector<vector<double> > A;
	vector< double > k;
	vector< vector <double> > Rt;
	vector< vector< vector <double> > > Rts;

	vector<cv::Mat> internal_images;
	vector<cv::Mat> external_images;
	vector<int> indices;

	VectorXd PA;
	VectorXd PB;


	int number_internal_images_written;


	cv::Size image_size;

	string text_file;

	CaliObjectOpenCV2(int i, int w, int h, double s_w_i, double s_h_i);

	void ReadImages(string internal_dir, bool flag);

	bool AccumulateCorners(bool draw_corners);

	bool AccumulateCornersFlexibleExternal(bool draw_corners);

	void Calibrate(std::ofstream& out, string write_directory);

	void CalibrateFlexibleExternal(std::ofstream& out, string write_directory);

	void LevMarCameraCaliNoDistortion(vector< vector<cv::Point2f> >& imagep, vector< vector<cv::Point3f> >& worldp,
			std::ofstream& out);

};


#endif /* ROBOTWORLDHANDEYECALIDUALEXP0_SRC_CALIBRATION2_HPP_ */
