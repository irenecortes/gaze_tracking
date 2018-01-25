
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
#include <geometry_msgs/PoseStamped.h>
#include </home/ivvi2018/catkin_ws/devel/include/tfm_msgs/Vector_Points.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

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

#define SHOW 1

#define ORIGINAL_RIGHT_EAR    0
#define ORIGINAL_MENTON       8
#define ORIGINAL_LEFT_EAR    16
#define ORIGINAL_RIGHT_BROW_1 17
#define ORIGINAL_RIGHT_BROW_2 18
#define ORIGINAL_RIGHT_BROW_3 19
#define ORIGINAL_RIGHT_BROW_4 20
#define ORIGINAL_RIGHT_BROW_5 21
#define ORIGINAL_LEFT_BROW_1 22
#define ORIGINAL_LEFT_BROW_2 23
#define ORIGINAL_LEFT_BROW_3 24
#define ORIGINAL_LEFT_BROW_4 25
#define ORIGINAL_LEFT_BROW_5 26
#define ORIGINAL_CTRE_EYES   27
#define ORIGINAL_NOSE        30
#define ORIGINAL_RIGHT_NOSE  31
#define ORIGINAL_CTR_NOSE    33
#define ORIGINAL_LEFT_NOSE   35
#define ORIGINAL_RIGHT_EYER  36
#define ORIGINAL_RIGHT_EYE_TOP_R 37
#define ORIGINAL_RIGHT_EYE_TOP_L 38
#define ORIGINAL_RIGHT_EYEL  39
#define ORIGINAL_RIGHT_EYE_BOTTOM_L 40
#define ORIGINAL_RIGHT_EYE_BOTTOM_R 41
#define ORIGINAL_LEFT_EYER  42
#define ORIGINAL_LEFT_EYE_TOP_R 43
#define ORIGINAL_LEFT_EYE_TOP_L 44
#define ORIGINAL_LEFT_EYEL  45
#define ORIGINAL_LEFT_EYE_BOTTOM_L 46
#define ORIGINAL_LEFT_EYE_BOTTOM_R 47
#define ORIGINAL_RIGHT_LIP_1 48
#define ORIGINAL_RIGHT_LIP_2 49
#define ORIGINAL_RIGHT_LIP_3 50
#define ORIGINAL_STOMMION    51
#define ORIGINAL_LEFT_LIP_1  52
#define ORIGINAL_LEFT_LIP_2  53
#define ORIGINAL_LEFT_LIP_3  54
#define ORIGINAL_BOTTOM_LIP_1 55
#define ORIGINAL_BOTTOM_LIP_2 56
#define ORIGINAL_BOTTOM_LIP_3 57
#define ORIGINAL_BOTTOM_LIP_4 58
#define ORIGINAL_BOTTOM_LIP_5 59
#define ORIGINAL_INNER_LIP_1 60
#define ORIGINAL_INNER_LIP_2 61
#define ORIGINAL_INNER_LIP_3 62
#define ORIGINAL_INNER_LIP_4 63
#define ORIGINAL_INNER_LIP_5 64
#define ORIGINAL_INNER_LIP_6 65


#define RIGHT_EAR    0
#define MENTON       1
#define LEFT_EAR     2
#define RIGHT_BROW_1 3
#define RIGHT_BROW_2 4
#define RIGHT_BROW_3 5
#define RIGHT_BROW_4 6
#define RIGHT_BROW_5 7
#define LEFT_BROW_1  8
#define LEFT_BROW_2  9
#define LEFT_BROW_3  10
#define LEFT_BROW_4  11
#define LEFT_BROW_5  12
#define CTRE_EYES    13
#define NOSE         14
#define RIGHT_NOSE   15
#define CTR_NOSE     16
#define LEFT_NOSE    17
#define RIGHT_EYER   18
#define RIGHT_EYE_TOP_R 19
#define RIGHT_EYE_TOP_L 20
#define RIGHT_EYEL   21
#define RIGHT_EYE_BOTTOM_L 22
#define RIGHT_EYE_BOTTOM_R 23
#define LEFT_EYER    24
#define LEFT_EYE_TOP_R 25
#define LEFT_EYE_TOP_L 26
#define LEFT_EYEL    27
#define LEFT_EYE_BOTTOM_L 28
#define LEFT_EYE_BOTTOM_R 29
#define RIGHT_LIP_1  30
#define RIGHT_LIP_2  31
#define RIGHT_LIP_3  32
#define STOMMION     33
#define LEFT_LIP_1   34
#define LEFT_LIP_2   35
#define LEFT_LIP_3   36
#define BOTTOM_LIP_1 37
#define BOTTOM_LIP_2 38
#define BOTTOM_LIP_3 39
#define BOTTOM_LIP_4 40
#define BOTTOM_LIP_5 41
#define INNER_LIP_1  42
#define INNER_LIP_2  43
#define INNER_LIP_3  44
#define INNER_LIP_4  45
#define INNER_LIP_5  46
#define INNER_LIP_6  47
#define RIGHT_PUPIL 48
#define LEFT_PUPIL 49

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
	dlib::point limsup_i;
	dlib::point liminf_i;
	dlib::point limsup_d;
	dlib::point liminf_d;
	dlib::point corner_izdo;
	dlib::point corner_dcho;
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

void DibujarRostro(cv::Mat &image, vector<Point2f> cara, Scalar color);
void DibujarEjes(cv::Mat &cam, cv::Mat & tran);
void proyectarRostro(Mat & image, vector<Point3f> & puntos_cara, vector<Point3f> & vector_p_ideales, Mat & tran);
void ObtienePuntos(std::vector<cv::Point3f> &Points3D, std::vector<cv::Point2i> &Points2D, std::vector<dlib::full_object_detection> & shapes, Face & driver, Mat & depth);
void Inicializar3D(std::vector<cv::Point3f> &Points3D);
void Inicializar3D_Real(std::vector<cv::Point3f> & Points3D);
void Inicializar2D_Real(std::vector<cv::Point2i> & Points2D);
void Inicializar3D_Ideal(std::vector<cv::Point3f> & Points3D);
void ObtieneTransfVec(Mat & transf, Mat & rotation, Mat & rvec, Mat & tvec);
void ObtieneTransfMat(Mat & transf, Mat & rotation, Mat & rvec, Mat & tvec);
void filter(Mat & rvec, Mat & tvec, vector<Mat> & rvecs, vector<Mat> & tvecs, Mat & rvec_f, Mat & tvec_f);
#define CTE 2.0
const static cv::Point3f P3D_SELLION    (0.0,  0.0,  0.0);
const static cv::Point3f P3D_RIGHT_EYER (-30*CTE,  9*CTE, 20*CTE);
const static cv::Point3f P3D_RIGHT_EYEL (-10*CTE,  9*CTE, 18*CTE);
const static cv::Point3f P3D_LEFT_EYER  ( 10*CTE,  9*CTE, 18*CTE);
const static cv::Point3f P3D_LEFT_EYEL  ( 30*CTE,  9*CTE, 20*CTE);
const static cv::Point3f P3D_NOSE       (  0*CTE, 30*CTE,-10*CTE);
const static cv::Point3f P3D_NOSEBR     (-10*CTE, 37*CTE,  7*CTE);
const static cv::Point3f P3D_NOSEBC     (  0.0,   37*CTE,   0.0);
const static cv::Point3f P3D_NOSEBL     ( 10*CTE, 37*CTE,  7*CTE);
const static cv::Point3f P3D_STOMMION   (  0.0,   46*CTE,   0.0);
const static cv::Point3f P3D_LIPR       (-15*CTE, 50*CTE,  10*CTE);
const static cv::Point3f P3D_LIPL       ( 15*CTE, 50*CTE,  10*CTE);
const static cv::Point3f P3D_MENTON     (  0.0,   75*CTE,   0.0);
#define N_RIGHT_EYER    0
#define N_RIGHT_EYEL    1
#define N_LEFT_EYER     2
#define N_LEFT_EYEL     3
#define N_NOSE          4
#define N_NOSEBR        5
#define N_NOSEBC        6
#define N_NOSEBL        7
#define N_STOMMION      8
#define N_LIPR          9
#define N_LIPL         10
#define N_MENTON       11

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

//void filtro_mediana(Mat rvec, Mat tvec, std::vector<cv::Point3f> & mediana){
//	vector<float> tvec_x, tvec_y, tvec_z;
//	vector<float> rvec_raw, p_vistos_y, p_vistos_z;
//	vector<float> corneas_x, corneas_y, corneas_z;
//
//
//	std::sort(pupilas_x.begin(), pupilas_x.end());
//	std::sort(pupilas_y.begin(), pupilas_y.end());
//	std::sort(pupilas_z.begin(), pupilas_z.end());
//
//	std::sort(corneas_x.begin(), corneas_x.end());
//	std::sort(corneas_y.begin(), corneas_y.end());
//	std::sort(corneas_z.begin(), corneas_z.end());
//
//	std::sort(p_vistos_x.begin(), p_vistos_x.end());
//	std::sort(p_vistos_y.begin(), p_vistos_y.end());
//	std::sort(p_vistos_z.begin(), p_vistos_z.end());
//
//	Point3f med (pupilas_x[pupilas_x.size() / 2], pupilas_y[pupilas_y.size() / 2], pupilas_z[pupilas_z.size() / 2]);
//	mediana.push_back(med);
//	med = Point3f(corneas_x[corneas_x.size() / 2], corneas_y[corneas_y.size() / 2], corneas_z[corneas_z.size() / 2]);
//	mediana.push_back(med);
//	med = Point3f(p_vistos_x[p_vistos_x.size() / 2], p_vistos_y[p_vistos_y.size() / 2], p_vistos_z[p_vistos_z.size() / 2]);
//	mediana.push_back(med);
//}
