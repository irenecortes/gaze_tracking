/*
 * camera_params.h
 *
 *  Created on: 18 dic. 2017
 *      Author: irenerrrd
 */

#ifndef FACE_DETECTION_INCLUDE_CAMERA_PARAMS_H_
#define FACE_DETECTION_INCLUDE_CAMERA_PARAMS_H_

#define QHD

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
/// Parámetros de calibración de las cámaras
#ifdef QHD
// Camara RGB
cv::Mat cameraMatrix_RGB = (Mat_<double>(3,3)
		<< 1054.33244474413 / 2.0, 0., 977.7452893998093 / 2.0,
		0., 1050.776182574851 / 2.0, 515.9194997769521 / 2.0,
		0., 0., 1.);

cv::Mat distCoeffs_RGB = (Mat_<double>(5,1)
		<< 0.03959418272099142 / 2.0,
		 -0.05511021582048477 / 2.0,
		 0.001148404869407374 / 2.0,
		 0.0007353074497996185 / 2.0,
		 0.01155286532375368 / 2.0);

#endif

#ifndef QHD

cv::Mat cameraMatrix_RGB = (Mat_<double>(3,3)
		<< 1054.33244474413, 0., 977.7452893998093,
		0., 1050.776182574851, 515.9194997769521,
		0., 0., 1.);

cv::Mat distCoeffs_RGB = (Mat_<double>(5,1)
		<< 0.03959418272099142,
		 -0.05511021582048477,
		 0.001148404869407374,
		 0.0007353074497996185,
		 0.01155286532375368);
#endif


cv::Mat cameraRot = (Mat_<double>(3,3)
		<<  0.9987934232925431, 0.02594937196487858, -0.04169325703505548,
		-0.02942533861183192, 0.9959426278606537, -0.08504370322131104,
		 0.03931726128740229, 0.08616792967597499, 0.9955045157407445);

cv::Mat cameraT_z = (Mat_<double>(3,1)
		<< 0, 0, 0);

cv::Mat cameraT = (Mat_<double>(3,1)
		<< -0.02269376426630372, 0.07450367527444389, 0.02745890706891858);

cv::Mat cameraTrans = (Mat_<double>(4,4)
		<<   0.9987934232925431, 0.02594937196487858, -0.04169325703505548, -0.02269376426630372,
		-0.02942533861183192, 0.9959426278606537, -0.08504370322131104,  0.07450367527444389,
		 0.03931726128740229, 0.08616792967597499, 0.9955045157407445, 0.02745890706891858,
		 0, 0, 0, 1);


Mat distCoeffs_null = Mat::zeros(5,1,CV_64FC1);




#endif /* FACE_DETECTION_INCLUDE_CAMERA_PARAMS_H_ */
