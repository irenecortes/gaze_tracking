/*
 * gaze_tracking.h
 *
 *  Created on: 22 nov. 2017
 *      Author: irenerrrd
 */

#ifndef FACE_DETECTION_INCLUDE_GAZE_TRACKING_H_
#define FACE_DETECTION_INCLUDE_GAZE_TRACKING_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>


#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include </home/ivvi2018/catkin_ws/devel/include/tfm_msgs/Vector_Points.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <camera_params.h>

using namespace cv;
using namespace std;
//using namespace dlib;

#define PI 3.141592
#define pimedios 1.570796
#define dospi 6.28318530718

#define degree2radian 0.01745329252
#define radian2degree 57.2958279

#define MIN_DEPTH 0.0
#define MAX_DEPTH 3000
#define MORPH_ELEM MORPH_ELLIPSE // Elipse kernel
#define MORPH_SIZE 3 // Kernel size

/// Imagen
#define IMAGE_W 960
#define IMAGE_H 540

#define CIRCLE1 1.85
#define CIRCLE2 2.6

// Filtrado HSV
#define MIN_V 40.0
#define MIN_S 0.1
#define MAX_S 0.6
#define MIN_H 22
#define MAX_H 168


#define SHOW 1

struct dir_ojo{
	Point3d cornea;
	Point3d pupila;
	Point3d punto_visto;
};

struct ojo{
	int presencia;
	int x;
	int y;
	Point2i pupila;
	Point3d pupila_w;
	int r; //radio iris
	Point2i limsup_i;
	Point2i liminf_i;
	Point2i limsup_d;
	Point2i liminf_d;
	Point2i corner_izdo;
	Point2i corner_dcho;
	Point3d limsup_i_w;
	Point3d liminf_i_w;
	Point3d limsup_d_w;
	Point3d liminf_d_w;
	Point3d corner_izdo_w;
	Point3d corner_dcho_w;
	float angx;
	float angy;
	vector<dir_ojo> puntos_vista;
};

struct Face{
	int presencia; //ok
	float x0;
	float y0;
	float z0;
	int offset_face_x;
	int offset_face_y;
	int face_w;
	int face_h;
	int offset_nose_x;
	int offset_nose_y;
	int nose_w;
	int nose_h;
	int offset_leye_x;
	int offset_leye_y;
	int leye_w;
	int leye_h;
	int offset_reye_x;
	int offset_reye_y;
	int reye_w;
	int reye_h;
	float roll;
	float yaw;
	float pitch;
	float t[3];
	struct ojo ojo_izdo;
	struct ojo ojo_dcho;
};

//INICIALIZACION

bool calibrationDone = false;
bool calib1 = false;
bool calib2 = false;
bool calib3 = false;
bool calib4 = false;
bool calib5 = false;
int calibCount = 0;
// subject-dependent personal parameters initialization
double alpha = 0.022;//-0.009; //-0.026; // First component of kappa angle (horizontal) - degrees
double betta = 0.082;//0.012;//-0.075; // Second component of kappa angle (vertical) - degrees
//double re = 0;//12.326; //14.1 /*/ 1000.0*/; // Eyeball radius - mm
//rce = Math.Sqrt(((Math.Pow(12.25,2))-(Math.Pow(6,2)))); // (Pitagoras th.)
double rce = 5.897; // Distance between cornea center and eyeball center - mm
Point3d vhe_izq (34.33, 23.534, 20.072);
//(30.96, 18.4, 74.3);// (29.5 /*/ 1000.0*/, 49.8 /*/ 1000.0*/, 57.5 /*/ 1000.0*/); // Eyeball center position in head coordinates system - mm
Point3d vhe_dcho (-34.33, 23.534, 20.072);
//(-30.96, 18.4, 74.3); //(-29.5 /*/ 1000.0*/, 49.8 /*/ 1000.0*/, 57.5 /*/ 1000.0*/);

float eye_aspect_ratio(struct ojo & p_ojo);
void DireccionOjo(struct ojo &p_ojo, int iz_o_dcho, Mat & tran, Point top_left_corner, Mat &cam, std::vector<dir_ojo> v_dir_ojo);
void DibujarEjes(Mat & cam, Mat & tran);
void GazeTracking(ojo &ojo, Point3d vhe, Mat & tran, Mat & image, Point3d & cornea, Point3d & visual_axis);
void DibujarOjos(cv::Mat &image, vector<Point2i> ojos, Scalar color);
void ObtieneTransfVec(Mat & rvec, Mat & tvec, Mat & transf);
Point2d CalculatePOR(double plane_A, double plane_B, double plane_C, double plane_D, Point3d & c, Point3d & nv, Mat & image);

#define RIGHT_EYER      0
#define RIGHT_EYE_TOP_R 1
#define RIGHT_EYE_TOP_L 2
#define RIGHT_EYEL      3
#define RIGHT_EYE_BOTTOM_L 4
#define RIGHT_EYE_BOTTOM_R 5
#define LEFT_EYER       6
#define LEFT_EYE_TOP_R  7
#define LEFT_EYE_TOP_L  8
#define LEFT_EYEL       9
#define LEFT_EYE_BOTTOM_L 10
#define LEFT_EYE_BOTTOM_R 11
#define RIGHT_PUPIL 12
#define LEFT_PUPIL 13

/*
Las dimensiones del globo ocular son:
- diámetro transversal: 23,5 mm
- diámetro vertical:    23 mm
- diámetro anteroposterior (de delante del ojo hacia la parte de atrás) de entre 22 a 24 mm.
 */

#define DIM_OJO 24
#define DIM_IRIS 13.0
#define OFFSET_VERT_OJO 0
#define OJO_DCHO 0
#define OJO_IZDO 1


#define EYE_AR_THRESH 0.25
#define EYE_AR_CONSEC_FRAMES 3
/// PCL
// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;




// UTILIDADES //
/*
 *  Devuelve la distancia entre dos puntos a y b en 2D
 */
float distanceP2P(Point a, Point b) {
	float d = sqrt(fabs(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
	return d;
}

/*
 *  Devuelve el angulo entre dos puntos a y b en 2D
 */
float angleP2P(Point a, Point b){
	float angle = atan2(a.y - b.y, a.x - b.x);

	if (angle < 0.0) {
		angle += dospi;
	}
	return (angle)/dospi*360;
}

/*
 *  Devuelve la distancia entre dos puntos a y b en 3D
 */
float distanceP3P(Point3d a, Point3d b) {
	float d = sqrt(fabs(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)+ pow(a.z - b.z, 2)));
	return d;
}

/*
 * Devuelve el angulo de entrada entre [-pi, pi]
 */
float norm_angulo(float angulo){
	float norm;
	if (angulo < PI){
		norm = angulo;
		if (angulo < -PI){
			norm = dospi + angulo;
		}
	} else {
		norm = -( dospi - angulo);
	}
	return norm;
}

double DotProduct (Point3d a, Point3d b){
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Point3d VectorialProduct(Point3d a, Point3d b){
	Point3d result;

	result.x = a.y * b.z - a.z * b.y;
	result.y = (a.x * b.z - a.z * b.x) * (-1.0);
	result.z = a.x * b.y - a.y * b.x;

	return result;
}

Point3d VectorialAddition(Point3d a, Point3d b){
	Point3d result;
	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;
	return result;
}

Point3d Dif3D (Point3d a, Point3d b){
	Point3d result;
	result.x = a.x - b.x;
	result.y = a.y - b.y;
	result.z = a.z - b.z;
	return result;
}

double Module(Point3d a){
	return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

#endif /* FACE_DETECTION_INCLUDE_GAZE_TRACKING_H_ */
