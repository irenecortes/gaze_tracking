
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <mutex>
#include <thread>


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
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include </home/ivvi2018/catkin_ws/devel/include/tfm_msgs/Vector_Points.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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

#define EYE_AR_THRESH 0.25
#define EYE_AR_CONSEC_FRAMES 3

/// Imagen
#define IMAGE_W 960
#define IMAGE_H 540

#define SHOW 1

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

struct dir_ojo{
	cv::Point3f cornea;
	cv::Point3f pupila;
	cv::Point3f punto_visto;
};

struct ojo{
	int presencia;
	int x;
	int y;
	float x_w;
	float y_w;
	float z_w;
	int r;
	Point2i limsup_i;
	Point2i liminf_i;
	Point2i limsup_d;
	Point2i liminf_d;
	Point2i corner_izdo;
	Point2i corner_dcho;
	cv::Point3f limsup_i_w;
	cv::Point3f liminf_i_w;
	cv::Point3f limsup_d_w;
	cv::Point3f liminf_d_w;
	cv::Point3f corner_izdo_w;
	cv::Point3f corner_dcho_w;
	float angx;
	float angy;
	vector<dir_ojo> puntos_vista;
};


void DibujarOjos(cv::Mat &image, vector<Point2i> ojos, Scalar color);
void DibujarEjes(cv::Mat &cam, cv::Mat & tran);
float eye_aspect_ratio(ojo & p_ojo);

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
float distanceP3P(Point3f a, Point3f b) {
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
