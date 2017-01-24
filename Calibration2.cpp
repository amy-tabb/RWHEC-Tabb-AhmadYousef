/*
 * Calibration2.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: atabb
 */
#include "Calibration2.hpp"
#include "StringFunctions.hpp"

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/legacy/legacy.hpp>

CaliObjectOpenCV2::CaliObjectOpenCV2(int i, int w, int h,  double s_w_i, double s_h_i){

	chess_w = w;
	chess_h = h;
	number_internal_images_written = 0;
	mean_reproj_error = 0;
	mm_height = s_h_i;
	mm_width = s_w_i;
	image_size =  cvSize(0, 0);
	text_file = "";

}


void CaliObjectOpenCV2::ReadImages(string internal_dir, bool flag){
	cv::Mat im;
	vector<string> im_names;
	string filename;
	string txt_ext = "txt";

	int count;


	im_names.clear();

	//string temp_dir = internal_dir + "/images";
	//string temp_dir =  internal_dir + cali_objects[i]->GUID;
	//	current_dir = write_dir + "/Cam" + ToString<int>(i)	+ "/external";
	cout << "Current dir " << internal_dir << endl;
	ReadDirectory(internal_dir, im_names);


	for (int c = 0; c < int(im_names.size()); c++){
		filename = internal_dir + "/" + im_names[c];
		cout << "Image name " << im_names[c] << endl;
		if (filename.size() > 3 && filename.substr(filename.size() - 3, filename.size()) != txt_ext){
			cout << "Reading filename .... " << filename << endl;
			im = cv::imread(filename.c_str());
			if (flag == 0){
				internal_images.push_back(im);
			}	else {
				external_images.push_back(im);
			}
		}	else {
			text_file = filename;
			cout << "Set filename! " << text_file << endl;
			//char ch; cin >> ch;
		}
	}

	//	cali_objects[i]->AccumulateCorners(write_dir);
}

bool CaliObjectOpenCV2::AccumulateCorners(bool draw_corners){

	//IplImage        *cimage = 0,		*gimage = 0, *result = 0;
	cv::Mat im, gimage, result;
	string current_name;
	bool corner_found;
	bool some_found = false;
	string filename;
	char ch;
	int corner_count = chess_h*chess_w;

	vector<cv::Point2f> pointBuf;
	cv::Point2f first_point, last_point;
	vector<cv::Point2f> flipped_points(corner_count);

	cv::Size boardsize;
	boardsize.height = chess_h;
	boardsize.width = chess_w;
	//bool

	//	string text_name;
	//	ofstream out;
	//	text_name = dir + "/data" + ToString<int>(camera_number) + ".txt";
	//	out.open(text_name.c_str());


	//for (int i = 0; i < int(images_to_process.size()); i++){
	number_internal_images_written = 0;
	cout << "Doing internal images now .... " << endl;
	for (int i = 0; i <  int(internal_images.size()); i++){
		cout << "Looking for corners " << i << endl;
		//for (int i = 0; i < int(images_to_process.size()); i++){
		// find corners

		im = internal_images[i];
		cv::cvtColor(im, gimage, CV_BGR2GRAY);

		corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf,  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);

		if (corner_found) {

			// need to flip the orientation, possibly ...

			//corners[chess_w*chess_h];

			first_point = pointBuf[0];
			last_point = pointBuf[chess_w*chess_h - 1];

			if (first_point.y < last_point.y){
				//if (first_point.x > last_point.x){
				cout << "WRONG ORIENTATION! " << endl;


				for (int k=0; k<corner_count; k++) {

					flipped_points[k] = pointBuf[chess_w*chess_h - 1 - k];

				}

				pointBuf.swap(flipped_points);

			}

			some_found = true;
			// refine the corner positions
			cv::cornerSubPix( gimage, pointBuf, cv::Size(11,11),
					cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


			// draw detected corner points in the current frame
			cv::drawChessboardCorners(internal_images[i], boardsize, pointBuf, true);

			all_points.push_back(pointBuf);

			cout << "Number of patterns " << all_points.size() << endl;
			number_internal_images_written++;


		}
	}



	cout << "Doing external images now .... " << endl;
	//for (int i = 0; i < 8; i++){
	for (int i = 0; i <  int(external_images.size()); i++){
		cout << "Looking for corners " << i << endl;
		//for (int i = 0; i < int(images_to_process.size()); i++){
		// find corners

		im = external_images[i];
		cv::cvtColor(im, gimage, CV_BGR2GRAY);

		cv::imwrite("gray.png", gimage);

		corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf,  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);

		if (!corner_found){
			cout << "Trying default option " << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf);
		}

		if (!corner_found){
			cout << "Trying  option one" << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf, CV_CALIB_CB_NORMALIZE_IMAGE);
		}

		if (!corner_found){
			cout << "Trying  option two" << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf, CV_CALIB_CB_FILTER_QUADS);
		}

		if (!corner_found){
			cout << "Trying  option three" << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH);
		}

		if (corner_found) {

			// need to flip the orientation, possibly ...

			//corners[chess_w*chess_h];

			first_point = pointBuf[0];
			last_point = pointBuf[chess_w*chess_h - 1];

			if (first_point.y < last_point.y){
				//if (first_point.x > last_point.x){
				cout << "WRONG ORIENTATION! " << endl;


				for (int k=0; k<corner_count; k++) {

					flipped_points[k] = pointBuf[chess_w*chess_h - 1 - k];

				}

				pointBuf.swap(flipped_points);

			}

			some_found = true;
			// refine the corner positions
			cv::cornerSubPix( gimage, pointBuf, cv::Size(11,11),
					cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


			if (draw_corners){
				// draw detected corner points in the current frame
				cv::drawChessboardCorners(external_images[i], boardsize, cv::Mat(pointBuf), true);
			}

			all_points.push_back(pointBuf);
			//				for (int k=0; k<corner_count; k++) {
			//					//all_corners.push_back(corners[k]);
			//
			//					//out << corners[k].x << ", " << corners[k].y << endl;
			//
			//				}

			cout << "Number of patterns " << all_points.size() << endl;
		}	else {
			cout << "WARNING -- EXTERNAL NOT FOUND! " << endl;
			exit(1);
		}
	}


	if (external_images.size() > 0){
		image_size = external_images[0].size();
	}


	if (external_images.size() > 0){
		image_size = external_images[0].size();
	}

	return some_found;

}

bool CaliObjectOpenCV2::AccumulateCornersFlexibleExternal(bool draw_corners){

	//IplImage        *cimage = 0,		*gimage = 0, *result = 0;
	cv::Mat im, gimage, result;
	string current_name;
	bool corner_found;
	bool some_found = false;
	string filename;
	char ch;
	int corner_count = chess_h*chess_w;

	vector<cv::Point2f> pointBuf;
	cv::Point2f first_point, last_point;
	vector<cv::Point2f> flipped_points(corner_count);

	cv::Size boardsize;
	boardsize.height = chess_h;
	boardsize.width = chess_w;
	//bool

	//	string text_name;
	//	ofstream out;
	//	text_name = dir + "/data" + ToString<int>(camera_number) + ".txt";
	//	out.open(text_name.c_str());


	//for (int i = 0; i < int(images_to_process.size()); i++){
	number_internal_images_written = 0;
	cout << "Doing internal images now .... " << endl;
	for (int i = 0; i <  int(internal_images.size()); i++){
		cout << "Looking for corners " << i << endl;
		//for (int i = 0; i < int(images_to_process.size()); i++){
		// find corners

		im = internal_images[i];
		cv::cvtColor(im, gimage, CV_BGR2GRAY);

		corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf,  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);

		if (corner_found) {

			// need to flip the orientation, possibly ...

			//corners[chess_w*chess_h];

			first_point = pointBuf[0];
			last_point = pointBuf[chess_w*chess_h - 1];

			if (first_point.y < last_point.y){
				//if (first_point.x > last_point.x){
				cout << "WRONG ORIENTATION! " << endl;


				for (int k=0; k<corner_count; k++) {

					flipped_points[k] = pointBuf[chess_w*chess_h - 1 - k];

				}

				pointBuf.swap(flipped_points);

			}

			some_found = true;
			// refine the corner positions
			cv::cornerSubPix( gimage, pointBuf, cv::Size(11,11),
					cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


			if (draw_corners){
				// draw detected corner points in the current frame
				cv::drawChessboardCorners(internal_images[i], boardsize, pointBuf, true);
			}

			all_points.push_back(pointBuf);

			cout << "Number of patterns " << all_points.size() << endl;
			number_internal_images_written++;


		}
	}



	cout << "Doing external images now .... " << endl;
	//for (int i = 0; i < 8; i++){
	for (int i = 0; i <  int(external_images.size()); i++){
		cout << "Looking for corners " << i << endl;
		//for (int i = 0; i < int(images_to_process.size()); i++){
		// find corners

		im = external_images[i];
		cv::cvtColor(im, gimage, CV_BGR2GRAY);

		//cv::imwrite("gray.png", gimage);

		corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf,  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);

		if (!corner_found){
			cout << "Trying default option " << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf);
		}

		if (!corner_found){
			cout << "Trying  option one" << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf, CV_CALIB_CB_NORMALIZE_IMAGE);
		}

		if (!corner_found){
			cout << "Trying  option two" << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf, CV_CALIB_CB_FILTER_QUADS);
		}

		if (!corner_found){
			cout << "Trying  option three" << endl;
			corner_found = cv::findChessboardCorners(gimage, boardsize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH);
		}

		if (corner_found) {

			// need to flip the orientation, possibly ...

			//corners[chess_w*chess_h];

			first_point = pointBuf[0];
			last_point = pointBuf[chess_w*chess_h - 1];

			if (first_point.y < last_point.y){
				//if (first_point.x > last_point.x){
				cout << "WRONG ORIENTATION! " << endl;


				for (int k=0; k<corner_count; k++) {

					flipped_points[k] = pointBuf[chess_w*chess_h - 1 - k];

				}

				pointBuf.swap(flipped_points);

			}

			some_found = true;
			// refine the corner positions
			cv::cornerSubPix( gimage, pointBuf, cv::Size(11,11),
					cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


			if (draw_corners){
				// draw detected corner points in the current frame
				cv::drawChessboardCorners(external_images[i], boardsize, cv::Mat(pointBuf), true);
			}

			all_points.push_back(pointBuf);
			//				for (int k=0; k<corner_count; k++) {
			//					//all_corners.push_back(corners[k]);
			//
			//					//out << corners[k].x << ", " << corners[k].y << endl;
			//
			//				}

			cout << "Number of patterns " << all_points.size() << endl;
		}	else {
			all_points.push_back(vector<cv::Point2f>());
			//cout << "WARNING -- EXTERNAL NOT FOUND! " << endl;
			//exit(1);
		}
	}


	if (external_images.size() > 0){
		image_size = external_images[0].size();
	}


	if (external_images.size() > 0){
		image_size = external_images[0].size();
	}

	return some_found;

}


struct CameraCaliData{
	vector< vector<cv::Point2f> >* image_Points;
	vector< vector<cv::Point3f> >* world_Points;
	vector<vector<double> > ps;
	vector<double>  vals;
};

//void residual_camera_no_distortion(double *p, double *x, int m, int n, void *data){
//
//
//	CameraCaliData *dptr;
//	dptr=(CameraCaliData*)data;
//
//	double k0 = p[0];
//	double k1 = p[1];
//	double k2 = p[2];
//	double k3 = p[3];
//	double k4 = p[4];
//
//	int index = 0;
//	for (int cam = 0; cam < int(dptr->image_Points->size()); cam++){
//		double thetax = p[5 + cam*6];
//		double thetay = p[6 + cam*6];
//		double thetaz = p[7 + cam*6];
//		double t0 = p[8 + cam*6];
//		double t1 = p[9 + cam*6];
//		double t2 = p[10 + cam*6];
//
//		for (int i = 0; i < int(dptr->image_Points->at(cam).size()); i++){
//
//			double xim = dptr->image_Points->at(cam)[i].x;
//			double yim = dptr->image_Points->at(cam)[i].y;
//			double W0 = dptr->world_Points->at(cam)[i].x;
//			double W1 = dptr->world_Points->at(cam)[i].y;
//			double W2 = dptr->world_Points->at(cam)[i].z;
//
//			x[index] = ((k0*sin(thetay)-k1*sin(thetax)*cos(thetay)+k2*cos(thetax)*cos(thetay))*W2
//					+(k1*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//							+k2*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))
//							-k0*cos(thetay)*sin(thetaz))
//							*W1
//							+(k2*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//									+k1*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz))
//									+k0*cos(thetay)*cos(thetaz))
//									*W0+k2*t2+k1*t1+k0*t0)
//									/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//											+sin(thetax)*cos(thetaz))
//											*W1
//											+(sin(thetax)*sin(thetaz)
//													-cos(thetax)*sin(thetay)*cos(thetaz))
//													*W0+t2)
//													-xim;
//			index++;
//
//			x[index] = ((k4*cos(thetax)*cos(thetay)-k3*sin(thetax)*cos(thetay))*W2
//					+(k3*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//							+k4*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz)))
//							*W1
//							+(k4*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//									+k3*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz)))
//									*W0+k4*t2+k3*t1)
//									/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//											+sin(thetax)*cos(thetaz))
//											*W1
//											+(sin(thetax)*sin(thetaz)
//													-cos(thetax)*sin(thetay)*cos(thetaz))
//													*W0+t2)
//													-yim;
//			index++;
//
//		}
//	}
//
//	double val = 0;
//
//	for (int i = 0; i < n; i++){
//		val += x[index]*x[index];
//	}
//
//	cout << "val! " << val << endl;
//	dptr->vals.push_back(val);
//
//	vector<double> params;
//	for (int i = 0; i < m; i++){
//		params.push_back(p[i]);
//	}
//
//	dptr->ps.push_back(params);
//}
//
//
//void jacobian_camera_no_distortion(double *p, double *jac, int m, int n, void *data){
//
//
//	CameraCaliData *dptr;
//	dptr=(CameraCaliData*)data;
//
//	double k0 = p[0];
//	double k1 = p[1];
//	double k2 = p[2];
//	double k3 = p[3];
//	double k4 = p[4];
//
//	int number_cams = int(dptr->image_Points->size());
//	int index = 0;
//	for (int cam = 0; cam < int(dptr->image_Points->size()); cam++){
//		double thetax = p[5 + cam*6];
//		double thetay = p[6 + cam*6];
//		double thetaz = p[7 + cam*6];
//		double t0 = p[8 + cam*6];
//		double t1 = p[9 + cam*6];
//		double t2 = p[10 + cam*6];
//
//		for (int i = 0; i < int(dptr->image_Points->at(cam).size()); i++){
//
//			double xim = dptr->image_Points->at(cam)[i].x;
//			double yim = dptr->image_Points->at(cam)[i].y;
//			double W0 = dptr->world_Points->at(cam)[i].x;
//			double W1 = dptr->world_Points->at(cam)[i].y;
//			double W2 = dptr->world_Points->at(cam)[i].z;
//
//			// do the ks first ....
//			jac[index] = (sin(thetay)*W2-cos(thetay)*sin(thetaz)*W1+cos(thetay)*cos(thetaz)*W0
//					+t0)
//					/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//							+sin(thetax)*cos(thetaz))
//							*W1
//							+(sin(thetax)*sin(thetaz)
//									-cos(thetax)*sin(thetay)*cos(thetaz))
//									*W0+t2);
//			jac[index + 1] =      (-sin(thetax)*cos(thetay)*W2+(cos(thetax)*cos(thetaz)
//					-sin(thetax)*sin(thetay)*sin(thetaz))
//					*W1
//					+(cos(thetax)*sin(thetaz)
//							+sin(thetax)*sin(thetay)*cos(thetaz))
//							*W0+t1)
//							/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//									+sin(thetax)*cos(thetaz))
//									*W1
//									+(sin(thetax)*sin(thetaz)
//											-cos(thetax)*sin(thetay)*cos(thetaz))
//											*W0+t2);
//
//			jac[index + 2] = 1;
//			jac[index + 3] = 0;
//			jac[index + 4] = 0;
//
//			index = index + 5;
//
//			for (int j = 0; j < cam*6; j++){
//				jac[index] = 0;
//				index++;
//			}
//			// now for each camera ....
//			jac[index + 0] =   ((-k2*sin(thetax)*cos(thetay)-k1*cos(thetax)*cos(thetay))*W2
//					+(k2*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//							+k1*(-cos(thetax)*sin(thetay)*sin(thetaz)-sin(thetax)*cos(thetaz)))
//							*W1
//							+(k1*(cos(thetax)*sin(thetay)*cos(thetaz)-sin(thetax)*sin(thetaz))
//									+k2*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz)))
//									*W0)
//									/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//											+sin(thetax)*cos(thetaz))
//											*W1
//											+(sin(thetax)*sin(thetaz)
//													-cos(thetax)*sin(thetay)*cos(thetaz))
//													*W0+t2)
//													-(-sin(thetax)*cos(thetay)*W2+(cos(thetax)*cos(thetaz)
//															-sin(thetax)*sin(thetay)*sin(thetaz))
//															*W1
//															+(cos(thetax)*sin(thetaz)
//																	+sin(thetax)*sin(thetay)*cos(thetaz))
//																	*W0)
//																	*((k0*sin(thetay)-k1*sin(thetax)*cos(thetay)
//																			+k2*cos(thetax)*cos(thetay))
//																			*W2
//																			+(k1*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//																					+k2*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))
//																					-k0*cos(thetay)*sin(thetaz))
//																					*W1
//																					+(k2*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//																							+k1*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz))
//																							+k0*cos(thetay)*cos(thetaz))
//																							*W0+k2*t2+k1*t1+k0*t0)
//																							/pow((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//																									+sin(thetax)*cos(thetaz))
//																									*W1
//																									+(sin(thetax)*sin(thetaz)
//																											-cos(thetax)*sin(thetay)*cos(thetaz))
//																											*W0+t2), 2);
//			jac[index + 1] =  ((k1*sin(thetax)*sin(thetay)-k2*cos(thetax)*sin(thetay)
//					+k0*cos(thetay))
//					*W2
//					+(k0*sin(thetay)*sin(thetaz)-k1*sin(thetax)*cos(thetay)*sin(thetaz)
//							+k2*cos(thetax)*cos(thetay)*sin(thetaz))
//							*W1
//							+(-k0*sin(thetay)*cos(thetaz)+k1*sin(thetax)*cos(thetay)*cos(thetaz)
//									-k2*cos(thetax)*cos(thetay)*cos(thetaz))
//									*W0)
//									/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//											+sin(thetax)*cos(thetaz))
//											*W1
//											+(sin(thetax)*sin(thetaz)
//													-cos(thetax)*sin(thetay)*cos(thetaz))
//													*W0+t2)
//													-(-cos(thetax)*sin(thetay)*W2+cos(thetax)*cos(thetay)*sin(thetaz)*W1
//															-cos(thetax)*cos(thetay)*cos(thetaz)*W0)
//															*((k0*sin(thetay)-k1*sin(thetax)*cos(thetay)
//																	+k2*cos(thetax)*cos(thetay))
//																	*W2
//																	+(k1*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//																			+k2*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))
//																			-k0*cos(thetay)*sin(thetaz))
//																			*W1
//																			+(k2*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//																					+k1*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz))
//																					+k0*cos(thetay)*cos(thetaz))
//																					*W0+k2*t2+k1*t1+k0*t0)
//																					/pow((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//																							+sin(thetax)*cos(thetaz))
//																							*W1
//																							+(sin(thetax)*sin(thetaz)
//																									-cos(thetax)*sin(thetay)*cos(thetaz))
//																									*W0+t2), 2);
//
//			jac[index + 2] =  ((k2*(cos(thetax)*sin(thetay)*cos(thetaz)-sin(thetax)*sin(thetaz))
//					+k1*(-cos(thetax)*sin(thetaz)-sin(thetax)*sin(thetay)*cos(thetaz))
//					-k0*cos(thetay)*cos(thetaz))
//					*W1
//					+(k1*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//							+k2*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))
//							-k0*cos(thetay)*sin(thetaz))
//							*W0)
//							/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//									+sin(thetax)*cos(thetaz))
//									*W1
//									+(sin(thetax)*sin(thetaz)
//											-cos(thetax)*sin(thetay)*cos(thetaz))
//											*W0+t2)
//											-((cos(thetax)*sin(thetay)*cos(thetaz)-sin(thetax)*sin(thetaz))*W1
//													+(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))*W0)
//													*((k0*sin(thetay)-k1*sin(thetax)*cos(thetay)
//															+k2*cos(thetax)*cos(thetay))
//															*W2
//															+(k1*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//																	+k2*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))
//																	-k0*cos(thetay)*sin(thetaz))
//																	*W1
//																	+(k2*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//																			+k1*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz))
//																			+k0*cos(thetay)*cos(thetaz))
//																			*W0+k2*t2+k1*t1+k0*t0)
//																			/pow((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//																					+sin(thetax)*cos(thetaz))
//																					*W1
//																					+(sin(thetax)*sin(thetaz)
//																							-cos(thetax)*sin(thetay)*cos(thetaz))
//																							*W0+t2), 2);
//
//			jac[index + 3] =  k0/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//					+sin(thetax)*cos(thetaz))
//					*W1
//					+(sin(thetax)*sin(thetaz)
//							-cos(thetax)*sin(thetay)*cos(thetaz))
//							*W0+t2);
//
//			jac[index + 4] =  k1/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//					+sin(thetax)*cos(thetaz))
//					*W1
//					+(sin(thetax)*sin(thetaz)
//							-cos(thetax)*sin(thetay)*cos(thetaz))
//							*W0+t2);
//
//			jac[index + 5] =  k2/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//					+sin(thetax)*cos(thetaz))
//					*W1
//					+(sin(thetax)*sin(thetaz)
//							-cos(thetax)*sin(thetay)*cos(thetaz))
//							*W0+t2)
//							-((k0*sin(thetay)-k1*sin(thetax)*cos(thetay)
//									+k2*cos(thetax)*cos(thetay))
//									*W2
//									+(k1*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//											+k2*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))
//											-k0*cos(thetay)*sin(thetaz))
//											*W1
//											+(k2*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//													+k1*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz))
//													+k0*cos(thetay)*cos(thetaz))
//													*W0+k2*t2+k1*t1+k0*t0)
//													/((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//															+sin(thetax)*cos(thetaz))
//															*W1
//															+(sin(thetax)*sin(thetaz)
//																	-cos(thetax)*sin(thetay)*cos(thetaz))
//																	*W0+t2), 2);
//			index = index + 6;
//
//			for (int j = (cam+ 1)*6; j < number_cams*6; j++){
//				jac[index] = 0;
//				index++;
//			}
//
//
//
//			////////////////////// now for r_y .......
//			// do the ks first ....
//			jac[index] = 0;
//			jac[index + 1]  = 0;
//			jac[index + 2] = 0;
//			jac[index + 3] = (-sin(thetax)*cos(thetay)*W2+(cos(thetax)*cos(thetaz)
//					-sin(thetax)*sin(thetay)*sin(thetaz))
//					*W1
//					+(cos(thetax)*sin(thetaz)
//							+sin(thetax)*sin(thetay)*cos(thetaz))
//							*W0+t1)
//							/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//									+sin(thetax)*cos(thetaz))
//									*W1
//									+(sin(thetax)*sin(thetaz)
//											-cos(thetax)*sin(thetay)*cos(thetaz))
//											*W0+t2);
//			jac[index + 4] = 1;
//
//			index = index + 5;
//
//			for (int j = 0; j < cam*6; j++){
//				jac[index] = 0;
//				index++;
//			}
//			// now for each camera ....
//			jac[index + 0] =        ((-k4*sin(thetax)*cos(thetay)-k3*cos(thetax)*cos(thetay))*W2
//					+(k4*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//							+k3*(-cos(thetax)*sin(thetay)*sin(thetaz)-sin(thetax)*cos(thetaz)))
//							*W1
//							+(k3*(cos(thetax)*sin(thetay)*cos(thetaz)-sin(thetax)*sin(thetaz))
//									+k4*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz)))
//									*W0)
//									/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//											+sin(thetax)*cos(thetaz))
//											*W1
//											+(sin(thetax)*sin(thetaz)
//													-cos(thetax)*sin(thetay)*cos(thetaz))
//													*W0+t2)
//													-(-sin(thetax)*cos(thetay)*W2+(cos(thetax)*cos(thetaz)
//															-sin(thetax)*sin(thetay)*sin(thetaz))
//															*W1
//															+(cos(thetax)*sin(thetaz)
//																	+sin(thetax)*sin(thetay)*cos(thetaz))
//																	*W0)
//																	*((k4*cos(thetax)*cos(thetay)-k3*sin(thetax)*cos(thetay))*W2
//																			+(k3*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//																					+k4*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz)))
//																					*W1
//																					+(k4*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//																							+k3*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz)))
//																							*W0+k4*t2+k3*t1)
//																							/pow((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//																									+sin(thetax)*cos(thetaz))
//																									*W1
//																									+(sin(thetax)*sin(thetaz)
//																											-cos(thetax)*sin(thetay)*cos(thetaz))
//																											*W0+t2), 2);
//			jac[index + 1]  =  ((k3*sin(thetax)*sin(thetay)-k4*cos(thetax)*sin(thetay))*W2
//					+(k4*cos(thetax)*cos(thetay)*sin(thetaz)
//							-k3*sin(thetax)*cos(thetay)*sin(thetaz))
//							*W1
//							+(k3*sin(thetax)*cos(thetay)*cos(thetaz)
//									-k4*cos(thetax)*cos(thetay)*cos(thetaz))
//									*W0)
//									/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//											+sin(thetax)*cos(thetaz))
//											*W1
//											+(sin(thetax)*sin(thetaz)
//													-cos(thetax)*sin(thetay)*cos(thetaz))
//													*W0+t2)
//													-((k4*cos(thetax)*cos(thetay)-k3*sin(thetax)*cos(thetay))*W2
//															+(k3*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//																	+k4*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz)))
//																	*W1
//																	+(k4*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//																			+k3*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz)))
//																			*W0+k4*t2+k3*t1)
//																			*(-cos(thetax)*sin(thetay)*W2+cos(thetax)*cos(thetay)*sin(thetaz)*W1
//																					-cos(thetax)*cos(thetay)*cos(thetaz)
//																					*W0)
//																					/pow((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//																							+sin(thetax)*cos(thetaz))
//																							*W1
//																							+(sin(thetax)*sin(thetaz)
//																									-cos(thetax)*sin(thetay)*cos(thetaz))
//																									*W0+t2), 2);
//
//			jac[index + 2] =  ((k4*(cos(thetax)*sin(thetay)*cos(thetaz)-sin(thetax)*sin(thetaz))
//					+k3*(-cos(thetax)*sin(thetaz)-sin(thetax)*sin(thetay)*cos(thetaz)))
//					*W1
//					+(k3*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//							+k4*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz)))
//							*W0)
//							/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//									+sin(thetax)*cos(thetaz))
//									*W1
//									+(sin(thetax)*sin(thetaz)
//											-cos(thetax)*sin(thetay)*cos(thetaz))
//											*W0+t2)
//											-((cos(thetax)*sin(thetay)*cos(thetaz)-sin(thetax)*sin(thetaz))*W1
//													+(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz))*W0)
//													*((k4*cos(thetax)*cos(thetay)-k3*sin(thetax)*cos(thetay))*W2
//															+(k3*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//																	+k4*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz)))
//																	*W1
//																	+(k4*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//																			+k3*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz)))
//																			*W0+k4*t2+k3*t1)
//																			/pow((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//																					+sin(thetax)*cos(thetaz))
//																					*W1
//																					+(sin(thetax)*sin(thetaz)
//																							-cos(thetax)*sin(thetay)*cos(thetaz))
//																							*W0+t2), 2);
//
//
//
//			jac[index + 3] = 0;
//
//			jac[index + 4] =  k3/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//					+sin(thetax)*cos(thetaz))
//					*W1
//					+(sin(thetax)*sin(thetaz)
//							-cos(thetax)*sin(thetay)*cos(thetaz))
//							*W0+t2);
//			jac[index + 5] =  k4/(cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//					+sin(thetax)*cos(thetaz))
//					*W1
//					+(sin(thetax)*sin(thetaz)
//							-cos(thetax)*sin(thetay)*cos(thetaz))
//							*W0+t2)
//							-((k4*cos(thetax)*cos(thetay)-k3*sin(thetax)*cos(thetay))*W2
//									+(k3*(cos(thetax)*cos(thetaz)-sin(thetax)*sin(thetay)*sin(thetaz))
//											+k4*(cos(thetax)*sin(thetay)*sin(thetaz)+sin(thetax)*cos(thetaz)))
//											*W1
//											+(k4*(sin(thetax)*sin(thetaz)-cos(thetax)*sin(thetay)*cos(thetaz))
//													+k3*(cos(thetax)*sin(thetaz)+sin(thetax)*sin(thetay)*cos(thetaz)))
//													*W0+k4*t2+k3*t1)
//													/pow((cos(thetax)*cos(thetay)*W2+(cos(thetax)*sin(thetay)*sin(thetaz)
//															+sin(thetax)*cos(thetaz))
//															*W1
//															+(sin(thetax)*sin(thetaz)
//																	-cos(thetax)*sin(thetay)*cos(thetaz))
//																	*W0+t2), 2);
//
//			index = index + 6;
//
//			for (int j = (cam+ 1)*6; j < number_cams*6; j++){
//				jac[index] = 0;
//				index++;
//			}
//
//
//		}
//	}
//}
//
//
//void CaliObjectOpenCV2::LevMarCameraCaliNoDistortion(vector< vector<cv::Point2f> >& imagep, vector< vector<cv::Point3f> >& worldp,
//		std::ofstream& out){
//
//	// Here, we let X by X inverse ... so we need to reprocess before leaving ....
//	char ch;
//
//	CameraCaliData C;
//	C.image_Points = &imagep;
//	C.world_Points = &worldp;
//
//	int number_cameras = worldp.size();
//	cout << "number camera " << number_cameras << endl;
//	cin >> ch;
//	// each image has world_points.size() items ....
//	int n = C.world_Points->size()* C.world_Points->at(0).size();
//	int m = 5 + 6*number_cameras;   // 5 camera params, then 6 compoenents for each image
//
//	double x[n];
//
//	//double jac[n*m];
//
//	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
//
//	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
//	opts[4]= LM_DIFF_DELTA; // relevant only if the Jacobian is approximated using finite differences; specifies forward differencing
//
//	// fill in using CO and undistort ....
//	int counter = 0;
//
//
//	for (int i = 0; i < n; i++){
//		x[i] = 0;
//	}
//
//
//	// convert X, Z, into parameter vector -- LATER
//
//	double parameter_vector[n];
//	for (int i = 0; i < m; i++){
//		parameter_vector[i] = 0;
//	}
//
//	parameter_vector[0] = 2500;
//	parameter_vector[2] = image_size.width/2.0;
//	parameter_vector[3] = 2500;
//	parameter_vector[4] = image_size.height/2.0;
//
//
//
//	out << endl;
//	out << "Intial solution ";
//	for(int i=0; i<m; ++i){
//		out <<	parameter_vector[i] << " ";
//	}
//	out << endl << endl;
//
//	//int ret;
//	int ret = dlevmar_der(residual_camera_no_distortion, jacobian_camera_no_distortion, parameter_vector, x, m, n, 1000, opts, info, NULL, NULL, &C); // with analytic Jacobian
//	//int ret;
//
//
//	//printf("Results for %s:\n", probname[problem]);
//	printf("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);
//	for(int i=0; i<m; ++i)
//		printf("%.7g ", parameter_vector[i]);
//	printf("\n\nMinimization info:\n");
//	for(int i=0; i<LM_INFO_SZ; ++i)
//		printf("%g ", info[i]);
//	printf("\n");
//
//	out << "Levenberg-Marquardt returned " << ret << " in " << info[5] << " iter, reason " << info[6] << endl;
//	out << " Solution ";
//	for(int i=0; i<m; ++i){
//		out <<	parameter_vector[i] << " ";
//	}
//	out << endl << endl;
//
//	out << "minimization information " << endl;
//	for(int i=0; i<LM_INFO_SZ; ++i){
//		out << info[i] << " ";
//	}
//	out << endl << endl;
//
//	//R = Rx*Ry*Rz;
//	// X params first, then Y params.
////	Matrix Rx(3, 3);
////	Matrix Ry(3, 3);
////	Matrix Rz(3, 3);
////
////
////	Rx.Row(1) << 1 <<  0 << 0;
////	Rx.Row(2) << 0 << cos(parameter_vector[0]) << -sin(parameter_vector[0]);
////	Rx.Row(3) << 0 << sin(parameter_vector[0]) << cos(parameter_vector[0]);
////
////	Ry.Row(1) << cos(parameter_vector[1]) << 0 << sin(parameter_vector[1]);
////	Ry.Row(2) << 0 << 1 << 0;
////	Ry.Row(3) << -sin(parameter_vector[1]) << 0 << cos(parameter_vector[1]);
////
////	Rz.Row(1) << cos(parameter_vector[2]) << -sin(parameter_vector[2]) << 0;
////	Rz.Row(2) << sin(parameter_vector[2]) << cos(parameter_vector[2]) << 0;
////	Rz.Row(3) << 0 << 0 << 1;
////
////
////	X.SubMatrix(1, 3, 1, 3) = Rx*Ry*Rz;
////	X(1, 4) = parameter_vector[3];
////	X(2, 4) = parameter_vector[4];
////	X(3, 4) = parameter_vector[5];
////
////	Rx.Row(1) << 1 <<  0 << 0;
////	Rx.Row(2) << 0 << cos(parameter_vector[6]) << -sin(parameter_vector[6]);
////	Rx.Row(3) << 0 << sin(parameter_vector[6]) << cos(parameter_vector[6]);
////
////	Ry.Row(1) << cos(parameter_vector[7]) << 0 << sin(parameter_vector[7]);
////	Ry.Row(2) << 0 << 1 << 0;
////	Ry.Row(3) << -sin(parameter_vector[7]) << 0 << cos(parameter_vector[7]);
////
////	Rz.Row(1) << cos(parameter_vector[8]) << -sin(parameter_vector[8]) << 0;
////	Rz.Row(2) << sin(parameter_vector[8]) << cos(parameter_vector[8]) << 0;
////	Rz.Row(3) << 0 << 0 << 1;
////
////
////	Z.SubMatrix(1, 3, 1, 3) = Rx*Ry*Rz;
////	Z(1, 4) = parameter_vector[9];
////	Z(2, 4) = parameter_vector[10];
////	Z(3, 4) = parameter_vector[11];
//
//
//
//}


void CaliObjectOpenCV2::Calibrate(std::ofstream& out, string write_directory){

	// need to make the points 3D

	vector< cv::Point3f> corners(chess_h*chess_w);
	string filename;


	cout << "Calibrating using " << all_points.size() << " patterns " << endl;

	int counter = 0;
	for( int i = 0; i < chess_h; ++i ){
		for( int j = 0; j < chess_w; ++j, counter++ ){
			corners[counter] = (cv::Point3f(float( j*mm_width ), float( i*mm_height ), 0));
		}
	}

	// b/c all of the positions are the same ....
	for (int i = 0; i < int(all_points.size()); i++){
		all_3d_corners.push_back(corners);
	}

	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
	//cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	vector<cv::Mat> rvecs, tvecs;

	if (image_size.width > 640){
		cameraMatrix.at<double>(0, 0) = 2500;
		cameraMatrix.at<double>(1, 1) = 2500;
	}	else {
		cameraMatrix.at<double>(0, 0) = 1800;
		cameraMatrix.at<double>(1, 1) = 1800;
		cameraMatrix.at<double>(0, 0) = 1000;
		cameraMatrix.at<double>(1, 1) = 1000;
	}
	cameraMatrix.at<double>(0, 2) = image_size.width/2;
	cameraMatrix.at<double>(1, 2) = image_size.height/2;

	cout << "initial camera matrix " << endl;

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			cout << cameraMatrix.at<double>(i, j) << " ";
		}
		cout << endl;
	}

	cout << "Running calibration " << endl;
	cout << "Number of dist coefficients  = " << distCoeffs.rows << endl;
	//LevMarCameraCaliNoDistortion(all_points, all_3d_corners, out);
	//double rms = cv::calibrateCamera(all_3d_corners, all_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS,
	//		cv::TermCriteria( cv::TermCriteria::COUNT, 2, DBL_EPSILON) );  //, CV_CALIB_RATIONAL_MODEL );
	//CV_CALIB_FIX_PRINCIPAL_POINT |
	//double rms = cv::calibrateCamera(all_3d_corners, all_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
	//		CV_CALIB_USE_INTRINSIC_GUESS| CV_CALIB_RATIONAL_MODEL);


	// this one does not use the initial guess
	double rms = 0;

	if (text_file.size() == 0){
		rms = cv::calibrateCamera(all_3d_corners, all_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
				CV_CALIB_RATIONAL_MODEL);

	}	else {
		ifstream in(text_file.c_str());
		string temp;
		in >> cameraMatrix.at<double>(0, 0);
		in >> temp;
		in >> cameraMatrix.at<double>(0, 2);
		in >> temp;
		in >> cameraMatrix.at<double>(1, 1);
		in >> cameraMatrix.at<double>(1, 2);
		in >> temp >> temp >> temp;
		in.close();
	}


	//double rms = cv::calibrateCamera(all_3d_corners, all_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS);

	cout << "rms " << rms << endl;
	cout << "camera matrix " << endl;


	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			cout << cameraMatrix.at<double>(i, j) << " ";
		}
		cout << endl;
	}

	cout << "Distortion " << endl;
	for (int i = 0; i < distCoeffs.rows; i++){
		cout << distCoeffs.at<double>(i, 0) << " ";
	}
	cout << endl;


	out << "Internal images used: " << number_internal_images_written << endl;
	out << "External images used: " << external_images.size() << endl;
	out << "rms " << rms << endl;
	out << "camera matrix " << endl;
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			out << cameraMatrix.at<double>(i, j) << " ";
		}
		out << endl;
	}

	out << "Distortion " << endl;
	for (int i = 0; i < distCoeffs.rows; i++){
		out << distCoeffs.at<double>(i, 0) << " ";
	}
	out << endl;

	//	vector<vector<double> > A;
	//		vector< double > k;
	//		vector< vector <double> > Rt;
	//		vector< vector< vector <double> > > Rts;

	cv::Mat rotMatrix = cv::Mat::eye(3, 3, CV_64F);
	vector< vector <double> > tempRt(3, vector<double>(4, 0));

	A.resize(3, vector<double>(3, 0));
	k.resize(8, 0);
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			A[i][j] = cameraMatrix.at<double>(i, j);
		}
	}

	for (int i = 0; i < distCoeffs.rows; i++){
		k[i] = distCoeffs.at<double>(i, 0);
	}


	double reproj_error = 0;
	vector<cv::Point2f> imagePoints2;
	double err;

	for (int m = number_internal_images_written; m < int(all_points.size()); m++){

		cv::projectPoints( cv::Mat(all_3d_corners[m]), rvecs[m], tvecs[m], cameraMatrix,  // project
				distCoeffs, imagePoints2);
		err = cv::norm(cv::Mat(all_points[m]), cv::Mat(imagePoints2), CV_L2);              // difference

		reproj_error        += err*err;                                             // su
	}

	out << endl << "Summed reproj error " << reproj_error << endl << endl;

	// we only want these for the external images ....
	for (int m = number_internal_images_written; m < int(all_points.size()); m++){
		cv::Rodrigues(rvecs[m], rotMatrix);

		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				tempRt[i][j] = rotMatrix.at<double>(i, j);
			}

			tempRt[i][3] = tvecs[m].at<double>(i);
		}

		Rts.push_back(tempRt);

		out << "Rt for cam " << m << endl;
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 4; j++){
				out << tempRt[i][j] << " ";
			}
			out << endl;
		}

		out << endl;
	}

	cout << "Finish with cali ... " << endl;

	cv::Mat view, rview, map1, map2;
	cv::Mat gray;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
			cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_size, 1, image_size, 0),
			image_size, CV_16SC2, map1, map2);

	for (int i = 0; i < int(external_images.size()); i++){
		cout << "Writing external " << i << endl;
		cv::remap(external_images[i], rview, map1, map2, cv::INTER_LINEAR);

		//cv::cvtColor(rview, gray, CV_BGR2GRAY);
		//cv::cvtColor(gray, rview, CV_GRAY2BGR);
		filename  = write_directory + "/ext" + ToString<int>(i) + ".png";
		cv::imwrite(filename.c_str(), rview);
	}
}

void CaliObjectOpenCV2::CalibrateFlexibleExternal(std::ofstream& out, string write_directory){

	// need to make the points 3D

	vector< cv::Point3f> corners(chess_h*chess_w);
	string filename;


	cout << "Calibrating using " << all_points.size() << " patterns " << endl;

	int counter = 0;
	for( int i = 0; i < chess_h; ++i ){
		for( int j = 0; j < chess_w; ++j, counter++ ){
			corners[counter] = (cv::Point3f(float( j*mm_width ), float( i*mm_height ), 0));
		}
	}

	// b/c all of the positions are the same ....
	// create map ....
	vector<bool> pattern_detected;
	vector<int> mapping_from_limited_to_full;
	vector< vector< cv::Point2f> > all_points_wo_blanks;

	for (int i = 0; i < int(all_points.size()); i++){
		if (all_points[i].size() > 0){
			mapping_from_limited_to_full.push_back(i);
			all_points_wo_blanks.push_back(all_points[i]);
			all_3d_corners.push_back(corners);
			pattern_detected.push_back(true);
		}	else {
			pattern_detected.push_back(false);
		}

	}

	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
	//cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	vector<cv::Mat> rvecs, tvecs;

	if (image_size.width > 640){
		cameraMatrix.at<double>(0, 0) = 2500;
		cameraMatrix.at<double>(1, 1) = 2500;
	}	else {
		cameraMatrix.at<double>(0, 0) = 1800;
		cameraMatrix.at<double>(1, 1) = 1800;
		cameraMatrix.at<double>(0, 0) = 1000;
		cameraMatrix.at<double>(1, 1) = 1000;

		cameraMatrix.at<double>(0, 0) = 600;
		cameraMatrix.at<double>(1, 1) = 600;
	}
	cameraMatrix.at<double>(0, 2) = image_size.width/2;
	cameraMatrix.at<double>(1, 2) = image_size.height/2;

	cout << "initial camera matrix " << endl;

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			cout << cameraMatrix.at<double>(i, j) << " ";
		}
		cout << endl;
	}

	cout << "Running calibration " << endl;
	cout << "Number of dist coefficients  = " << distCoeffs.rows << endl;
	//LevMarCameraCaliNoDistortion(all_points, all_3d_corners, out);
	//double rms = cv::calibrateCamera(all_3d_corners, all_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS,
	//		cv::TermCriteria( cv::TermCriteria::COUNT, 2, DBL_EPSILON) );  //, CV_CALIB_RATIONAL_MODEL );
	//CV_CALIB_FIX_PRINCIPAL_POINT |
	//double rms = cv::calibrateCamera(all_3d_corners, all_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
	//		CV_CALIB_USE_INTRINSIC_GUESS| CV_CALIB_RATIONAL_MODEL);


	// this one does not use the initial guess
	//double rms = cv::calibrateCamera(all_3d_corners, all_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
	//		CV_CALIB_RATIONAL_MODEL);

	double rms = 0;
	char ch;
//	cout << "Before first " << endl; cin >> ch;
//	rms = cv::calibrateCamera(all_3d_corners, all_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
//							CV_CALIB_RATIONAL_MODEL);


	if (text_file.size() == 0){
		// submitted Transactions paper has rational model
		//rms = cv::calibrateCamera(all_3d_corners, all_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
		//		CV_CALIB_RATIONAL_MODEL);

		rms = cv::calibrateCamera(all_3d_corners, all_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs);

	}	else {
		ifstream in(text_file.c_str());
		string temp;
		in >> cameraMatrix.at<double>(0, 0);
		in >> temp;
		in >> cameraMatrix.at<double>(0, 2);
		in >> temp;
		in >> cameraMatrix.at<double>(1, 1);
		in >> cameraMatrix.at<double>(1, 2);
		in >> temp >> temp >> temp;
		in.close();

		cout << "initial camera matrix " << endl;

		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				cout << cameraMatrix.at<double>(i, j) << " ";
			}
			cout << endl;
		}

//		cout << "Before first " << endl; cin >> ch;
//		rms = cv::calibrateCamera(all_3d_corners, all_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
//						CV_CALIB_RATIONAL_MODEL);

		//cout << "Before second " << endl; cin >> ch;
		// submitted Transactions paper has rational model
		//rms = cv::calibrateCamera(all_3d_corners, all_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
		//		CV_CALIB_RATIONAL_MODEL| CV_CALIB_USE_INTRINSIC_GUESS);

		rms = cv::calibrateCamera(all_3d_corners, all_points_wo_blanks, image_size, cameraMatrix, distCoeffs, rvecs, tvecs,
						CV_CALIB_USE_INTRINSIC_GUESS);
	}



	//double rms = cv::calibrateCamera(all_3d_corners, all_points, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS);

	cout << "rms " << rms << endl;
	cout << "camera matrix " << endl;


	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			cout << cameraMatrix.at<double>(i, j) << " ";
		}
		cout << endl;
	}

	cout << "Distortion " << endl;
	for (int i = 0; i < distCoeffs.rows; i++){
		cout << distCoeffs.at<double>(i, 0) << " ";
	}
	cout << endl;


	out << "Internal images used: " << number_internal_images_written << endl;
	out << "External images used: " << external_images.size() << endl;
	out << "rms " << rms << endl;
	out << "camera matrix " << endl;
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			out << cameraMatrix.at<double>(i, j) << " ";
		}
		out << endl;
	}

	out << "Distortion " << endl;
	for (int i = 0; i < distCoeffs.rows; i++){
		out << distCoeffs.at<double>(i, 0) << " ";
	}
	out << endl;

	cv::Mat rotMatrix = cv::Mat::eye(3, 3, CV_64F);
	vector< vector <double> > tempRt(3, vector<double>(4, 0));

	A.resize(3, vector<double>(3, 0));
	k.resize(8, 0);
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			A[i][j] = cameraMatrix.at<double>(i, j);
		}
	}

	for (int i = 0; i < distCoeffs.rows; i++){
		k[i] = distCoeffs.at<double>(i, 0);
	}
	cout << "Line 13378 " << endl;
	//char ch; cin >> ch;

	double reproj_error = 0;
	vector<cv::Point2f> imagePoints2;
	double err;

	for (int m = number_internal_images_written; m < int(all_points_wo_blanks.size()); m++){

		cv::projectPoints( cv::Mat(all_3d_corners[m]), rvecs[m], tvecs[m], cameraMatrix,  // project
				distCoeffs, imagePoints2);
		err = cv::norm(cv::Mat(all_points_wo_blanks[m]), cv::Mat(imagePoints2), CV_L2);              // difference
		reproj_error        += err*err;


		// su
	}

	out << endl << "Summed reproj error " << reproj_error << endl << endl;

	Matrix temp;
	for (int m = number_internal_images_written; m < int(all_points.size()); m++){
		Rts.push_back(vector<vector <double> >());
	}

//	cout << "Line 1364 " << endl;
//	cout << "number internal images written " << number_internal_images_written << endl;
//	cout << "Number of total pattterns " << all_points_wo_blanks.size() << endl;
//	cout << "Number rvecs " << rvecs.size() << endl;
//	cout << "Number Rts " << Rts.size() << endl;
//	cin >> ch;
	// we only want these for the external images ....
	for (int m = number_internal_images_written; m < int(all_points_wo_blanks.size()); m++){


		cv::Rodrigues(rvecs[m], rotMatrix);
		//cout << "after conversion " << endl; cin >> ch;

		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++){
				tempRt[i][j] = rotMatrix.at<double>(i, j);
			}

			tempRt[i][3] = tvecs[m].at<double>(i);
		}
//		cout << "after tempRT " << endl; cin >> ch;
//
//		cout << "mapping " << mapping_from_limited_to_full.at(m) << endl; cin >> ch;

		//Rts.push_back(tempRt);
		Rts[mapping_from_limited_to_full.at(m) - number_internal_images_written] = tempRt;
		//cout << "RTs " << endl; cin >> ch;

		out << "Rt for cam " << mapping_from_limited_to_full[m] - number_internal_images_written << endl;
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 4; j++){
				out << tempRt[i][j] << " ";
			}
			out << endl;
		}

		out << endl;
	}

	cout << "Finish with cali ... " << endl;
	//cin >> ch;

	cv::Mat view, rview, map1, map2;
	cv::Mat gray;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
			cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image_size, 1, image_size, 0),
			image_size, CV_16SC2, map1, map2);

	for (int i = 0; i < int(external_images.size()); i++){
		cout << "Writing external " << i << endl;
		cv::remap(external_images[i], rview, map1, map2, cv::INTER_LINEAR);

		//cv::cvtColor(rview, gray, CV_BGR2GRAY);
		//cv::cvtColor(gray, rview, CV_GRAY2BGR);
		filename  = write_directory + "/ext" + ToString<int>(i) + ".png";
		cv::imwrite(filename.c_str(), rview);
	}
}
