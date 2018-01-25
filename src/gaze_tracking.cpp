
#include "gaze_tracking.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
Mat imageRGB; // depthData,
ojo ojo_izquierdo, ojo_derecho;
Mat rvec, tvec;
vector<Point2i> puntos_ojos_2d;

//void get_points3 (const tfm_msgs::Vector_Points& input) {
//	vector<Point3d> puntos_ojos_3d;
//	//	cout << "get_points " << input.num_puntos.data << endl;
//	for (int i = 0; i < input.num_puntos.data; i++){
//		Point3d punto_ojos;
//		punto_ojos.x = input.puntos[i].x;
//		punto_ojos.y = input.puntos[i].y;
//		punto_ojos.z = input.puntos[i].z;
//		puntos_ojos_3d.push_back(punto_ojos);
//	}
//
//	ojo_izquierdo.corner_dcho_w = puntos_ojos_3d[LEFT_EYER];
//	ojo_izquierdo.corner_izdo_w = puntos_ojos_3d[LEFT_EYEL];
//	ojo_izquierdo.liminf_d_w = puntos_ojos_3d[LEFT_EYE_TOP_R];
//	ojo_izquierdo.liminf_i_w = puntos_ojos_3d[LEFT_EYE_TOP_L];
//	ojo_izquierdo.limsup_d_w = puntos_ojos_3d[LEFT_EYE_BOTTOM_R];
//	ojo_izquierdo.limsup_i_w = puntos_ojos_3d[LEFT_EYE_BOTTOM_L];
//	ojo_izquierdo.pupila_w = puntos_ojos_3d[LEFT_PUPIL];
//	ojo_izquierdo.presencia = 1;
//
//	ojo_derecho.corner_dcho_w = puntos_ojos_3d[RIGHT_EYER];
//	ojo_derecho.corner_izdo_w = puntos_ojos_3d[RIGHT_EYEL];
//	ojo_derecho.liminf_d_w = puntos_ojos_3d[RIGHT_EYE_TOP_R];
//	ojo_derecho.liminf_i_w = puntos_ojos_3d[RIGHT_EYE_TOP_L];
//	ojo_derecho.limsup_d_w = puntos_ojos_3d[RIGHT_EYE_BOTTOM_R];
//	ojo_derecho.limsup_i_w = puntos_ojos_3d[RIGHT_EYE_BOTTOM_L];
//	ojo_derecho.pupila_w = puntos_ojos_3d[RIGHT_PUPIL];
//	ojo_derecho.presencia = 1;
//}
//
//
//void get_points2 (const tfm_msgs::Vector_Points& input) {
//	puntos_ojos_2d.clear();
//	//	cout << "get_points " << input.num_puntos.data << endl;
//	for (int i = 0; i < input.num_puntos.data; i++){
//		Point2i punto_ojos;
//		punto_ojos.x = input.puntos[i].x;
//		punto_ojos.y = input.puntos[i].y;
//		puntos_ojos_2d.push_back(punto_ojos);
//	}
//
//	ojo_izquierdo.corner_dcho = puntos_ojos_2d[LEFT_EYER];
//	ojo_izquierdo.corner_izdo = puntos_ojos_2d[LEFT_EYEL];
//	ojo_izquierdo.liminf_d = puntos_ojos_2d[LEFT_EYE_TOP_R];
//	ojo_izquierdo.liminf_i = puntos_ojos_2d[LEFT_EYE_TOP_L];
//	ojo_izquierdo.limsup_d = puntos_ojos_2d[LEFT_EYE_BOTTOM_R];
//	ojo_izquierdo.limsup_i = puntos_ojos_2d[LEFT_EYE_BOTTOM_L];
//	ojo_izquierdo.x = (ojo_izquierdo.corner_dcho.x + ojo_izquierdo.corner_izdo.x) / 2.0;
//	ojo_izquierdo.y = (ojo_izquierdo.corner_dcho.y + ojo_izquierdo.corner_izdo.y) / 2.0;
//	ojo_izquierdo.pupila = puntos_ojos_2d[LEFT_PUPIL];
//	ojo_izquierdo.presencia = 1;
//
//	ojo_derecho.corner_dcho = puntos_ojos_2d[RIGHT_EYER];
//	ojo_derecho.corner_izdo = puntos_ojos_2d[RIGHT_EYEL];
//	ojo_derecho.liminf_d = puntos_ojos_2d[RIGHT_EYE_TOP_R];
//	ojo_derecho.liminf_i = puntos_ojos_2d[RIGHT_EYE_TOP_L];
//	ojo_derecho.limsup_d = puntos_ojos_2d[RIGHT_EYE_BOTTOM_R];
//	ojo_derecho.limsup_i = puntos_ojos_2d[RIGHT_EYE_BOTTOM_L];
//	ojo_derecho.x = (ojo_derecho.corner_dcho.x + ojo_derecho.corner_izdo.x) / 2.0;
//	ojo_derecho.y = (ojo_derecho.corner_dcho.y + ojo_derecho.corner_izdo.y) / 2.0;
//	ojo_derecho.pupila = puntos_ojos_2d[RIGHT_PUPIL];
//	ojo_derecho.presencia = 1;
//}
//
//void get_Pose (const geometry_msgs::PoseStamped& input) {
//	tvec = Mat(3, 1, CV_64FC1, Scalar(0.0));
//	rvec = Mat(3, 1, CV_64FC1, Scalar(0.0));
//
//	tvec.at<double>(0) = input.pose.position.x;
//	tvec.at<double>(1) = input.pose.position.y;
//	tvec.at<double>(2) = input.pose.position.z;
//
//	tf::Quaternion q;
//	q.setX(input.pose.orientation.x);
//	q.setY(input.pose.orientation.y);
//	q.setZ(input.pose.orientation.z);
//	q.setW(input.pose.orientation.w);
//
//	tf::Matrix3x3 m(q);
//	double roll, pitch, yaw;
//	m.getRPY(roll, pitch, yaw);
//
//	//	tf::Vector3 v3 = q.getAxis();
//	rvec.at<double>(0) = roll; //v3[0];
//	rvec.at<double>(1) = pitch; // v3[1];
//	rvec.at<double>(2) = yaw; //v3[2];
//}

//void get_cloud (const sensor_msgs::PointCloud2ConstPtr& input) {
//	pcl::PointCloud<pcl::PointXYZRGB> cloud_aux;
//
//	pcl::fromROSMsg(*input, cloud_aux);
//	_cloud = cloud_aux.makeShared();
//
//	depthData = Mat(_cloud->height, _cloud->width, CV_32FC3, Scalar(0.0, 0.0, 0.0));
//
//	for (int i = 0, v = 0; v < _cloud->height; v++){
//		for (int u = 0; u < _cloud->width; u++, i++){
//			// Obtiene la información de profundidad de la nube de puntos
//			if (!isnan(_cloud->points[i].x)/* && cloud_aux.points[i].z < max*/)
//				depthData.at<Vec3f>(v,u)[0] = _cloud->points[i].x * 1000;
//			if (!isnan(_cloud->points[i].y)/* && cloud_aux.points[i].z < max*/)
//				depthData.at<Vec3f>(v,u)[1] = _cloud->points[i].y * 1000;
//			if (!isnan(_cloud->points[i].z)/* && cloud_aux.points[i].z < max*/)
//				depthData.at<Vec3f>(v,u)[2] = _cloud->points[i].z * 1000;
//		}
//	}
//}

//void getRGBimage (const sensor_msgs::Image& input){
//	cv_bridge::CvImagePtr img;
//	img = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8); // container of type sensor_msgs/Image
//	imageRGB = img->image;
//	//	imshow("Imagen", imageRGB);
//	//	waitKey(1);
//}

//void getDepthImage (const sensor_msgs::Image& input){
//	cv_bridge::CvImagePtr img;
//	img = cv_bridge::toCvCopy(input); // container of type sensor_msgs/Image
//	depthData = img->image;
//	//	imshow("Imagen", imageRGB);
//	//	waitKey(1);
//}

void callback(const geometry_msgs::PoseStampedConstPtr& pose,
		const tfm_msgs::Vector_PointsConstPtr& points1,
		const tfm_msgs::Vector_PointsConstPtr& points2/*,
		const sensor_msgs::ImageConstPtr& image*/) {

	tvec = Mat(3, 1, CV_64FC1, Scalar(0.0));
	rvec = Mat(3, 1, CV_64FC1, Scalar(0.0));

	tvec.at<double>(0) = pose->pose.position.x;
	tvec.at<double>(1) = pose->pose.position.y;
	tvec.at<double>(2) = pose->pose.position.z;

	tf::Quaternion q;
	q.setX(pose->pose.orientation.x);
	q.setY(pose->pose.orientation.y);
	q.setZ(pose->pose.orientation.z);
	q.setW(pose->pose.orientation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	//	tf::Vector3 v3 = q.getAxis();
	rvec.at<double>(0) = roll; //v3[0];
	rvec.at<double>(1) = pitch; // v3[1];
	rvec.at<double>(2) = yaw; //v3[2];



	vector<Point3d> puntos_ojos_3d;
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < points1->num_puntos.data; i++){
		Point3d punto_ojos;
		punto_ojos.x = points1->puntos[i].x;
		punto_ojos.y = points1->puntos[i].y;
		punto_ojos.z = points1->puntos[i].z;
		puntos_ojos_3d.push_back(punto_ojos);
	}

	ojo_izquierdo.corner_dcho_w = puntos_ojos_3d[LEFT_EYER];
	ojo_izquierdo.corner_izdo_w = puntos_ojos_3d[LEFT_EYEL];
	ojo_izquierdo.liminf_d_w = puntos_ojos_3d[LEFT_EYE_TOP_R];
	ojo_izquierdo.liminf_i_w = puntos_ojos_3d[LEFT_EYE_TOP_L];
	ojo_izquierdo.limsup_d_w = puntos_ojos_3d[LEFT_EYE_BOTTOM_R];
	ojo_izquierdo.limsup_i_w = puntos_ojos_3d[LEFT_EYE_BOTTOM_L];
	ojo_izquierdo.pupila_w = puntos_ojos_3d[LEFT_PUPIL];
	ojo_izquierdo.presencia = 1;

	ojo_derecho.corner_dcho_w = puntos_ojos_3d[RIGHT_EYER];
	ojo_derecho.corner_izdo_w = puntos_ojos_3d[RIGHT_EYEL];
	ojo_derecho.liminf_d_w = puntos_ojos_3d[RIGHT_EYE_TOP_R];
	ojo_derecho.liminf_i_w = puntos_ojos_3d[RIGHT_EYE_TOP_L];
	ojo_derecho.limsup_d_w = puntos_ojos_3d[RIGHT_EYE_BOTTOM_R];
	ojo_derecho.limsup_i_w = puntos_ojos_3d[RIGHT_EYE_BOTTOM_L];
	ojo_derecho.pupila_w = puntos_ojos_3d[RIGHT_PUPIL];
	ojo_derecho.presencia = 1;



	puntos_ojos_2d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < points2->num_puntos.data; i++){
		Point2i punto_ojos;
		punto_ojos.x = points2->puntos[i].x;
		punto_ojos.y = points2->puntos[i].y;
		puntos_ojos_2d.push_back(punto_ojos);
	}

	ojo_izquierdo.corner_dcho = puntos_ojos_2d[LEFT_EYER];
	ojo_izquierdo.corner_izdo = puntos_ojos_2d[LEFT_EYEL];
	ojo_izquierdo.liminf_d = puntos_ojos_2d[LEFT_EYE_TOP_R];
	ojo_izquierdo.liminf_i = puntos_ojos_2d[LEFT_EYE_TOP_L];
	ojo_izquierdo.limsup_d = puntos_ojos_2d[LEFT_EYE_BOTTOM_R];
	ojo_izquierdo.limsup_i = puntos_ojos_2d[LEFT_EYE_BOTTOM_L];
	ojo_izquierdo.x = (ojo_izquierdo.corner_dcho.x + ojo_izquierdo.corner_izdo.x) / 2.0;
	ojo_izquierdo.y = (ojo_izquierdo.corner_dcho.y + ojo_izquierdo.corner_izdo.y) / 2.0;
	ojo_izquierdo.pupila = puntos_ojos_2d[LEFT_PUPIL];
	ojo_izquierdo.presencia = 1;

	ojo_derecho.corner_dcho = puntos_ojos_2d[RIGHT_EYER];
	ojo_derecho.corner_izdo = puntos_ojos_2d[RIGHT_EYEL];
	ojo_derecho.liminf_d = puntos_ojos_2d[RIGHT_EYE_TOP_R];
	ojo_derecho.liminf_i = puntos_ojos_2d[RIGHT_EYE_TOP_L];
	ojo_derecho.limsup_d = puntos_ojos_2d[RIGHT_EYE_BOTTOM_R];
	ojo_derecho.limsup_i = puntos_ojos_2d[RIGHT_EYE_BOTTOM_L];
	ojo_derecho.x = (ojo_derecho.corner_dcho.x + ojo_derecho.corner_izdo.x) / 2.0;
	ojo_derecho.y = (ojo_derecho.corner_dcho.y + ojo_derecho.corner_izdo.y) / 2.0;
	ojo_derecho.pupila = puntos_ojos_2d[RIGHT_PUPIL];
	ojo_derecho.presencia = 1;

	/*cv_bridge::CvImagePtr img;
	img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); // container of type sensor_msgs/Image
	imageRGB = img->image;*/

}

int main (int argc, char** argv) {

	vhe_izq.x = atoi(argv[1]);
	vhe_izq.y = atoi(argv[2]);
	vhe_izq.z = atoi(argv[3]);

	vhe_dcho.x = -atoi(argv[1]);
	vhe_dcho.y = atoi(argv[2]);
	vhe_dcho.z = atoi(argv[3]);

	struct Face driver;
	int num_images = 0;
	std::vector<dir_ojo> dir_izquierdo;
	std::vector<dir_ojo> dir_derecho;

	Point3d topLeftCorner (140.6, -308.4, 6.8);
	Point3d topRightCorner (-336.1, -315.8,  20.9);
	Point3d bottomLeftCorner (135.0, -43.7,  -46.4);
	Point3d bottomRightCorner (-341.8, -51.1,  -32.2);

	//	Point3d topLeftCorner (125.0, -300.0, -70.0);
	//	Point3d topRightCorner (-333.0, -300.0,  -70.0);
	//	Point3d bottomLeftCorner (125.0, -40.0,  -70.0);
	//	Point3d bottomRightCorner (-333.0, -40.0,  -70.0);
	// Screen plane: Ax + By + Cz + D = 0
	// Point from the plane
	Point3d plane_p = topLeftCorner, plane_u = topRightCorner - topLeftCorner,  plane_v = bottomLeftCorner - topLeftCorner;
	// A, B, C, D of the equation
	double plane_A = plane_u.y * plane_v.z - plane_u.z * plane_v.y;
	double plane_B = plane_u.z * plane_v.x - plane_u.x * plane_v.z;
	double plane_C = plane_u.x * plane_v.y - plane_u.y * plane_v.x;
	double plane_D = - plane_A*plane_p.x - plane_B*plane_p.y - plane_C*plane_p.z;

	// cout << plane_A << ", " << plane_B << ", " << plane_C << ", " << plane_D << endl;

	try{
		//	 Initializa ROS
		ros::init (argc, argv, "gaze_tracking");
		ros::NodeHandle nh;
		std_msgs::String msg;

		//	 Crear subscriptores para la nube de puntos y la imagen de color
		//		ros::Subscriber sub_pose = nh.subscribe ("/face/pose", 1, get_Pose);
		//		ros::Subscriber sub_image = nh.subscribe ("/face/image_ROI", 1, getRGBimage);
		//		ros::Subscriber sub_puntos3 = nh.subscribe ("/eyes/points3d", 1, get_points3);
		//		ros::Subscriber sub_puntos2 = nh.subscribe ("/eyes/points2d", 1, get_points2);
		//		ros::Subscriber sub_image_depth = nh.subscribe ("/face/image_depth_ROI", 1, getDepthImage);
		//		ros::Subscriber sub_color = nh.subscribe ("/kinect2/qhd/image_color_rect", 1, getRGBimage);
		//		ros::Subscriber sub_points  = nh.subscribe ("/kinect2/qhd/points", 1, get_cloud);

		message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose(nh, "/face/pose", 1);
		message_filters::Subscriber<tfm_msgs::Vector_Points> sub_puntos3(nh, "/eyes/points3d", 1);
		message_filters::Subscriber<tfm_msgs::Vector_Points> sub_puntos2(nh, "/eyes/points2d", 1);
		//		message_filters::Subscriber<sensor_msgs::Image> sub_color(nh, "/kinect2/qhd/image_color_rect", 1);

		typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, tfm_msgs::Vector_Points, tfm_msgs::Vector_Points/*, sensor_msgs::Image*/> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_pose, sub_puntos3, sub_puntos2/*, sub_color*/);
		sync.registerCallback(boost::bind(&callback, _1, _2, _3/*, _4*/));



		std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
		double fps = 0;
		size_t frameCount = 0;
		std::ostringstream oss;

		//			Esperar hasta obtener los subscriptores
		//		while ((sub_pose.getNumPublishers() == 0) && (sub_color.getNumPublishers() == 0)) {
		//			ROS_ERROR("Waiting for publishers");
		//			sleep(1);
		//		}
		//		ROS_INFO("Got publisher");

		start = std::chrono::high_resolution_clock::now();
		ojo_derecho.presencia = 0;
		ojo_izquierdo.presencia = 0;

		ros::Rate r(20); // 10 hz
		while (nh.ok()) {
			//		 Spin
			ros::spinOnce();

			//			if (!imageRGB.empty() && !depthData.empty()){
#ifndef QHD
			Mat image_dibujar(1080, 1920, CV_8UC3, Scalar(0, 0, 0));
#endif
#ifdef QHD
			Mat image_dibujar(540, 960, CV_8UC3, Scalar(0, 0, 0));
#endif
			//imageRGB.clone();
			//				Point2d tlc (roi_anterior.x, roi_anterior.y);
			//				if(ojo_derecho.presencia && ojo_izquierdo.presencia){
			////					Point2d right_pupil, left_pupil;
			////					PupilDetection(ojo_izquierdo, ojo_derecho, right_pupil, left_pupil, imageRGB);
			//
			//					if (right_pupil != Point2d(0,0)){
			//						//						cout << right_pupil << endl;
			//						circle(image_dibujar, right_pupil, 1, cv::Scalar(255, 0, 255), -1);
			//					}
			//
			//					if (left_pupil != Point2d(0,0)){
			//						//						cout << left_pupil << endl;
			//						circle(image_dibujar, left_pupil, 1, cv::Scalar(255, 0, 255), -1);
			//					}
			//				}

			DibujarOjos(image_dibujar, puntos_ojos_2d, Scalar(255, 255, 0));
			Mat transf;
			if (!rvec.empty() && !tvec.empty()){
				ObtieneTransfVec(rvec, tvec, transf);
				DibujarEjes(image_dibujar, transf);
				//					cout << "TVEC: " << tvec << endl;
			}

			Point3d cornea_dcha, cornea_izda, direccion_vista_dcha, direccion_vista_izda;
			if(ojo_derecho.presencia){
				//					ojo_derecho.pupila_w.x = depthData.at<Vec3f>(ojo_derecho.pupila.y, ojo_derecho.pupila.x)[0];
				//					ojo_derecho.pupila_w.y = depthData.at<Vec3f>(ojo_derecho.pupila.y, ojo_derecho.pupila.x)[1];
				//					ojo_derecho.pupila_w.z = depthData.at<Vec3f>(ojo_derecho.pupila.y, ojo_derecho.pupila.x)[2];
				if (!rvec.empty() && !tvec.empty())
					GazeTracking(ojo_derecho, vhe_dcho, transf, image_dibujar, cornea_dcha, direccion_vista_dcha);
			}
			//
			if(ojo_izquierdo.presencia){
				//					ojo_izquierdo.pupila_w.x = depthData.at<Vec3f>(ojo_izquierdo.pupila.y, ojo_izquierdo.pupila.x)[0];
				//					ojo_izquierdo.pupila_w.y = depthData.at<Vec3f>(ojo_izquierdo.pupila.y, ojo_izquierdo.pupila.x)[1];
				//					ojo_izquierdo.pupila_w.z = depthData.at<Vec3f>(ojo_izquierdo.pupila.y, ojo_izquierdo.pupila.x)[2];
				if (!rvec.empty() && !tvec.empty())
					GazeTracking(ojo_izquierdo, vhe_izq, transf, image_dibujar, cornea_izda, direccion_vista_izda);
			}

			Mat image_gaze(1080, 1920, CV_8UC3, Scalar(0,0,0));
			Rect ROI_imagen_dibujar(1920 / 2.0 - (image_dibujar.cols / 2.0), 1080 / 2.0 - (image_dibujar.rows / 2.0), image_dibujar.cols, image_dibujar.rows);
			//				cout << 1080 / 2.0 - image_dibujar.cols / 2.0 << ", " << 1920 / 2.0 - image_dibujar.rows / 2.0 << ", " << image_dibujar.cols << ", " << image_dibujar.rows << endl;
			if (ROI_imagen_dibujar.x >= 0 && ROI_imagen_dibujar.y >= 0 && ROI_imagen_dibujar.height > 0 && ROI_imagen_dibujar.width > 0){
				Point v1 (ROI_imagen_dibujar.x, ROI_imagen_dibujar.y);
				Point v2 (ROI_imagen_dibujar.x + ROI_imagen_dibujar.width, ROI_imagen_dibujar.y + ROI_imagen_dibujar.height);					//					image_dibujar.copyTo(image_gaze(ROI_imagen_dibujar), image_dibujar);
				//					cout << v1 << ", " << v2 << endl;
				//					rectangle(image_gaze, v1, v2, Scalar(0, 0, 255), 2);
				flip(image_dibujar, image_dibujar, 1);
				image_dibujar.copyTo(image_gaze(ROI_imagen_dibujar), image_dibujar);
			} else {
				imshow("Image_dibujar", image_dibujar);
			}

			Point2d POR_izdo = CalculatePOR(plane_A, plane_B, plane_C, plane_D, cornea_izda, direccion_vista_izda, image_gaze);
			Point2d POR_dcho = CalculatePOR(plane_A, plane_B, plane_C, plane_D, cornea_dcha, direccion_vista_dcha, image_gaze);

			//cout << "POR_izdo: " << POR_izdo << endl;
			//cout << "POR_dcho: " << POR_dcho << endl;

			//				resize(image_dibujar, image_dibujar, Size(1920, 1080));

			//				cv::putText(imageRGB, oss.str(), Point(700,30), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 255), 1, 8, false);


			circle(image_gaze, POR_izdo, 3, Scalar(0,0,255), 1);
			circle(image_gaze, POR_dcho, 3, Scalar(0,0,255), 1);
			circle(image_gaze, (POR_dcho + POR_izdo) / 2.0, 5, Scalar(0,0,255), -1);

			namedWindow("Gaze Tracking", CV_WINDOW_NORMAL);
			cv::imshow("Gaze Tracking", image_gaze);
			//				namedWindow("Cara detectada", CV_WINDOW_NORMAL);
			//				cv::imshow("Cara detectada", image_dibujar);
			waitKey(1);
			imageRGB.release();

			++frameCount;
			now = std::chrono::high_resolution_clock::now();
			double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
			if(elapsed >= 1.0)
			{
				fps = frameCount / elapsed;
				oss.str("");
				oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
				start = now;
				frameCount = 0;
			}
			//			}
			r.sleep();
		}

	} catch(dlib::serialization_error& e) {
		cout << "You need dlib's default face landmarking model file to run this example." << endl;
		cout << "You can get it from the following URL: " << endl;
		cout << "   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
		cout << endl << e.what() << endl;

	} catch(exception& e) {
		cout << e.what() << endl;
	}

	return(0);
}

//Obtiene la matriz de rotación y los vectores de rotación y traslación. Convierte la matriz de transformación en una 4x4 con la que se puede operar
void ObtieneTransfVec(Mat & rvec, Mat & tvec, Mat & transf){
	Mat rotation;
	Rodrigues(rvec, rotation);
	hconcat(rotation, tvec, transf);

	cv::Mat row = (Mat_<double>(1,4) << 0.0, 0.0, 0.0, 1.0);
	transf.push_back(row);
	//	cout << "RT recibida: " << transf << endl;
}


void DibujarEjes(cv::Mat &cam, cv::Mat & tran) {
	cv::Point3d Pto3D_camera;
	std::vector<cv::Point3d> PointsEjes;
	Mat Pto3D_H;

	//Se definen los puntos de los ejes tridimensionalmente y se transforman para que el origen de coordenadas
	//aparezca entre los ojos de la cara detectada
	Pto3D_H = (Mat_<double>(4,1) << 0, 0, 0, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0) / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1) / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2) / 1000.0;
	PointsEjes.push_back(Pto3D_camera);

	Pto3D_H = (Mat_<double>(4,1) << 100, 0, 0, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0) / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1) / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2) / 1000.0;
	PointsEjes.push_back(Pto3D_camera);

	Pto3D_H = (Mat_<double>(4,1) << 0, 100, 0, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0) / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1) / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2) / 1000.0;
	PointsEjes.push_back(Pto3D_camera);

	Pto3D_H = (Mat_<double>(4,1) << 0, 0, 100, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0) / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1) / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2) / 1000.0;
	PointsEjes.push_back(Pto3D_camera);

	Mat vhe_i = (Mat_<double>(4,1) << vhe_izq.x, vhe_izq.y, vhe_izq.z, 1);
	Point3d Pto3D_ojo_izd;
	Pto3D_H = tran*vhe_i;
	Pto3D_ojo_izd.x = Pto3D_H.at<double>(0) / 1000.0;
	Pto3D_ojo_izd.y = Pto3D_H.at<double>(1) / 1000.0;
	Pto3D_ojo_izd.z = Pto3D_H.at<double>(2) / 1000.0;
	PointsEjes.push_back(Pto3D_ojo_izd);

	Mat vhe_d = (Mat_<double>(4,1) << vhe_dcho.x, vhe_dcho.y, vhe_dcho.z, 1);
	Point3d Pto3D_ojo_dcho;
	Pto3D_H = tran*vhe_d;
	Pto3D_ojo_dcho.x = Pto3D_H.at<double>(0) / 1000.0;
	Pto3D_ojo_dcho.y = Pto3D_H.at<double>(1) / 1000.0;
	Pto3D_ojo_dcho.z = Pto3D_H.at<double>(2) / 1000.0;
	PointsEjes.push_back(Pto3D_ojo_dcho);

	//Se proyectan los puntos tridimensionales en la imagen
	std::vector<cv::Point2d>  Pto2DEjes;
	projectPoints(PointsEjes, /*cameraRot*/ cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, Pto2DEjes, cv::noArray(), 0);

	Point2i Pto2D_ojo_izd = Pto2DEjes.back();
	Pto2DEjes.pop_back();
	Point2i Pto2D_ojo_dcho = Pto2DEjes.back();
	Pto2DEjes.pop_back();
	circle(cam, Pto2D_ojo_izd , 3, cv::Scalar(255,255,255,0), 1);
	circle(cam, Pto2D_ojo_dcho, 3, cv::Scalar(255,255,255,0), 1);


	for (int i = 0; i < Pto2DEjes.size(); i++){
		Pto2DEjes[i] = Point(Pto2DEjes[i].x /*- top_left_corner.x*/, Pto2DEjes[i].y /*- top_left_corner.y*/);
		if (Pto2DEjes[i].x < 0 || Pto2DEjes[i].x > cam.cols || Pto2DEjes[i].y < 0 || Pto2DEjes[i].y > cam.rows)
			cout << "Eje fuera de imagen" << endl;
	}

	//Se dibujan los ejes en la imagen
	cv::line(cam, Pto2DEjes[0],Pto2DEjes[1], cv::Scalar(255,0,0,0), 1, 8, 0);
	putText(cam, "X", Pto2DEjes[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0), 1, 8, false);

	cv::line(cam, Pto2DEjes[0],Pto2DEjes[2], cv::Scalar(0,255,0,0), 1, 8, 0);
	putText(cam, "Y", Pto2DEjes[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0), 1, 8, false);

	cv::line(cam, Pto2DEjes[0],Pto2DEjes[3], cv::Scalar(0,0,255,0), 1, 8, 0);
	putText(cam, "Z", Pto2DEjes[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0), 1, 8, false);
}

void DibujarOjos(cv::Mat &image, vector<Point2i> ojos, Scalar color){
	//	cout << "DibujarRostro " << cara.size() << endl;
	// Around Chin. Ear to Ear
	//	for (unsigned long i = 1; i <= 16; ++i)
	//		cv::line(image, Point(shape.part(i).x(), shape.part(i).y()), Point(shape.part(i-1).x(), shape.part(i-1).y()), cv::Scalar(255,255,0,0), 1, 8, 0);

	if (ojos.size() > 11){

		// Left eye
		for (unsigned long i = RIGHT_EYE_TOP_R; i <= RIGHT_EYE_BOTTOM_R; ++i)
			cv::line(image, Point(ojos[i].x, ojos[i].y), Point(ojos[i-1].x, ojos[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(ojos[RIGHT_EYER].x, ojos[RIGHT_EYER].y), Point(ojos[RIGHT_EYE_BOTTOM_R].x, ojos[RIGHT_EYE_BOTTOM_R].y), color, 1, 8, 0);

		// Right eye
		for (unsigned long i = LEFT_EYE_TOP_R; i <= LEFT_EYE_BOTTOM_R; ++i)
			cv::line(image, Point(ojos[i].x, ojos[i].y), Point(ojos[i-1].x, ojos[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(ojos[LEFT_EYER].x, ojos[LEFT_EYER].y), Point(ojos[LEFT_EYE_BOTTOM_R].x, ojos[LEFT_EYE_BOTTOM_R].y), color, 1, 8, 0);

		if (ojos.size() > 12){
			circle(image, ojos[RIGHT_PUPIL], 1, cv::Scalar(255, 0, 255), -1);
			ojo_derecho.presencia = 1;
		}
		if (ojos.size() > 13){
			circle(image, ojos[LEFT_PUPIL], 1, cv::Scalar(255, 0, 255), -1);
			ojo_izquierdo.presencia = 1;
		}
	}
}


/*
void DireccionOjo(struct ojo &p_ojo, int iz_o_dcho, Mat & tran, Point top_left_corner, Mat &cam, std::vector<dir_ojo> v_dir_ojo) {
	Point3d pupila_world (p_ojo.x_w / 1000.0, p_ojo.y_w / 1000.0, p_ojo.z_w / 1000.0);
	Point3d cornea_real ((p_ojo.corner_dcho_w.x + p_ojo.corner_izdo_w.x) / 2.0,
			(p_ojo.corner_dcho_w.y + p_ojo.corner_izdo_w.y) / 2.0,
			(p_ojo.corner_dcho_w.z + p_ojo.corner_izdo_w.z) / 2.0);
	Mat cornea_real_H = (Mat_<double>(4,1) << cornea_real.x, cornea_real.y, cornea_real.z, 1);

	Mat prof = (Mat_<double>(4,4) << tran.at<double>(0,0), tran.at<double>(0,1), tran.at<double>(0,2), tran.at<double>(0,3),
			tran.at<double>(1,0), tran.at<double>(1,1), tran.at<double>(1,2), tran.at<double>(1,3),
			tran.at<double>(2,0), tran.at<double>(2,1), tran.at<double>(2,2), tran.at<double>(2,3) + 24.0,
			0, 0, 0, 1);

	//	cout << "Cornea antes: " << cornea_real_H << endl;
	cornea_real_H = tran.inv()*prof*cornea_real_H;
	//	cout << "Cornea después: " << cornea_real_H << endl;

	Point3d cornea_w;
	cornea_w.x = cornea_real_H.at<double>(0) / 1000.0;
	cornea_w.y = cornea_real_H.at<double>(1) / 1000.0;
	cornea_w.z = cornea_real_H.at<double>(2) / 1000.0;

	Point3d v_dir_recta (pupila_world.x - cornea_w.x, pupila_world.y - cornea_w.y, pupila_world.z - cornea_w.z);
	Point3d punto_mira (pupila_world.x + 20*v_dir_recta.x, pupila_world.y + 20*v_dir_recta.y, pupila_world.z + 20*v_dir_recta.z);
	//	cout << "Punto mira: " << punto_mira << endl;

	dir_ojo nueva_dir;
	nueva_dir.cornea = cornea_w;
	nueva_dir.pupila = pupila_world;
	nueva_dir.punto_visto = punto_mira;

	if (v_dir_ojo.size() >= 10){
		v_dir_ojo.erase(v_dir_ojo.begin());
	}
	v_dir_ojo.push_back(nueva_dir);
	vector<Point3d> v_p3d;
	filtro_mediana(v_dir_ojo, v_p3d);
	//	v_p3d.push_back(pupila_world);
	//	v_p3d.push_back(cornea_w);
	//	v_p3d.push_back(punto_mira);

#ifdef SHOW
	vector<Point2d> v_p2d;
	projectPoints(v_p3d, cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, v_p2d, cv::noArray(), 0);

	//	Point pupila2D (v_p2d[0].x + top_left_corner.x , v_p2d[0].y + top_left_corner.y);
	//	Point cornea2D (v_p2d[1].x + top_left_corner.x , v_p2d[1].y + top_left_corner.y);
	//	Point punto_mira2D (v_p2d[2].x + top_left_corner.x , v_p2d[2].y + top_left_corner.y);

	Point pupila2D (v_p2d[0].x , v_p2d[0].y );
	Point cornea2D (v_p2d[1].x , v_p2d[1].y );
	Point punto_mira2D (v_p2d[2].x , v_p2d[2].y );


	if (pupila2D.x < 0 || pupila2D.x > cam.cols || pupila2D.y < 0 || pupila2D.y > cam.rows)
		cout << "Pupila fuera de imagen" << endl;

	if (cornea2D.x < 0 || cornea2D.x > cam.cols || cornea2D.y < 0 || cornea2D.y > cam.rows)
		cout << "Cornea fuera de imagen" << endl;

	//	if (punto_mira2D.x < 0 || punto_mira2D.x > cam.cols || punto_mira2D.y < 0 || punto_mira2D.y > cam.rows)
	cout << "Punto mira fuera de imagen" << endl;

	line(cam, pupila2D, punto_mira2D, cv::Scalar(255,255,255,0), 2, cv::LINE_8, 0 );
	circle(cam, pupila2D/*cv::Point(p_ojo.x,p_ojo.y)* /, 3, cv::Scalar(255,255,255,0), -1, cv::LINE_8, 0 );
	circle(cam, cornea2D, 3, cv::Scalar(255,255,255,0), 1, cv::LINE_8, 0 );

#endif

}
 */


void GazeTracking(ojo &ojo, Point3d vhe, Mat & tran, Mat & image, Point3d & cornea, Point3d & visual_axis){
	// subject- dependent personal parameters
	//todo: CREO QUE ESTAN MAL LOS PUNTOS DE ORIGEN O ALGO ASI, LAS CUENTAS ESTÁN BIEN
	// vision parameters
	Point3d no; // Optical axis (vector)
	Point3d nv; // Visual axis (vector)
	Point3d c; // Cornea center
	// gaze_direction = c + parameter*nv;
	Point2d pointOfRegard; // POR
	//	cout << " 0 " ;
	Point3d p = ojo.pupila_w; // Pupil center
	//	Point3d r_vector_radians (rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
	//	Point3d t_vector (tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

	// e -> eyeball center in camera coordinates - mm
	// e = R*Vhe + T

	Mat vhe_d = (Mat_<double>(4,1) << vhe.x, vhe.y, vhe.z, 1);
	Point3d Pto3D_ojo_dcho;
	Mat e_point_mat = tran*vhe_d;
	Point3d e_point (e_point_mat.at<double>(0), e_point_mat.at<double>(1), e_point_mat.at<double>(2));
	//	Point3d e_point = VectorialProduct(r_vector_radians, vhe) + t_vector;
	//	cout << e_point << " ";

	// No = (p-e)/||p-e|| (mm)
	no = (p - e_point) / norm(p - e_point);
	//	no.x = (p - e_point).x / (Module(p - e_point));
	//	no.y = (p - e_point).y / (Module(p - e_point));
	//	no.z = (p - e_point).z / (Module(p - e_point));
	//	cout << no << " ";
	// c = e + rce*No (mm)
	c = rce * no + e_point;
	//	c.x = rce * no.x + e_point.x;
	//	c.y = rce * no.y + e_point.y;
	//	c.z = rce * no.z + e_point.z;
	//	cout << c << " ";
	// Nv
	double phi, gamma; // Multiplicar por Math.PI/180 para convertir los grados en radianes.
	phi = asin(no.y); //radians
	gamma = atan2(no.x, no.z); //radians
	//	cout << gamma << " ";
	// alpha and betta to radians
	nv.x = cos(phi + alpha) * sin(gamma + betta);
	nv.y = sin(phi + alpha);
	nv.z = cos(phi + alpha) * cos(gamma + betta);
	//	cout << nv << " ";
	// Gaze direction = c + lambda Nv

	//Debug.WriteLine("Pupil center -> X: " + p.x + " Y: " + p.y + " Z: " + p.z);
	//Debug.WriteLine("Eyeball center camera coordinates -> X: " + e_point.x + " Y: " + e_point.y + " Z: " + e_point.z);
	//Debug.WriteLine("Optical axis No -> X: " + no.x + " Y: " + no.y + " Z: " + no.z);
	//Debug.WriteLine("Cornea center c -> X: " + c.x + " Y: " + c.y + " Z: " + c.z);
	//Debug.WriteLine("Visual axis Nv -> X: " + nv.x + " Y: " + nv.y + " Z: " + nv.z);

	//Debug.WriteLine("Gaze direction: [" + c.x + "," + c.y + "," + c.z + "] + t * [" + nv.x + "," + nv.y + "," + nv.z + "]");

	vector<Point3d> Points_Gaze;
	// Draw gaze direction
	Point3d /*CameraSpacePoint*/ point0;
	point0.x = (double)c.x + 0 * (double)nv.x;
	point0.y = (double)c.y + 0 * (double)nv.y;
	point0.z = (double)c.z + 0 * (double)nv.z;
	//	cout << "P0: " << point0 << endl;
	Points_Gaze.push_back(point0 / 1000.0);
	Point3d /*CameraSpacePoint*/ point1;
	point1.x = (double)c.x + (double)100.0 * (double)nv.x;
	point1.y = (double)c.y + (double)100.0 * (double)nv.y;
	point1.z = (double)c.z + (double)100.0 * (double)nv.z;
	Points_Gaze.push_back(point1 / 1000.0);
	//	cout << "P1: " << point1 << endl;
	Point3d /*CameraSpacePoint*/ point2;
	point2.x = (double)c.x + (double)200.0 * (double)nv.x;
	point2.y = (double)c.y + (double)200.0 * (double)nv.y;
	point2.z = (double)c.z + (double)200.0 * (double)nv.z;
	Points_Gaze.push_back(point2 / 1000.0);
	//	cout << "P2: " << point2 << endl;
	Point3d /*CameraSpacePoint*/ point3;
	point3.x = (double)c.x + (double)300.0 * (double)nv.x;
	point3.y = (double)c.y + (double)300.0 * (double)nv.y;
	point3.z = (double)c.z + (double)300.0 * (double)nv.z;
	//	cout << "P3: " << point3 << endl;
	Points_Gaze.push_back(point3 / 1000.0);

	//	Points_Gaze.push_back(point0);
	std::vector<cv::Point2d>  Pto2DGaze;
	projectPoints(Points_Gaze, /*cameraRot*/ cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, Pto2DGaze, cv::noArray(), 0);

	for (int i = 0; i < (int)Pto2DGaze.size() - 1; i++){
		cv::line(image, Pto2DGaze[i],Pto2DGaze[i + 1], cv::Scalar(0,255,255,0), 1, 8, 0);
	}
	// cout << endl;

	cornea = c;
	visual_axis = nv;

	//Debug.WriteLine("POV0 -> X: " + point0.x + " Y: " + point0.y + " Z: " + point0.z);
	//Debug.WriteLine("POV1 -> X: " + point1.x + " Y: " + point1.y + " Z: " + point1.z);
	//Debug.WriteLine("POV2 -> X: " + point2.x + " Y: " + point2.y + " Z: " + point2.z);
	//Debug.WriteLine("POV3 -> X: " + point3.x + " Y: " + point3.y + " Z: " + point3.z);

	//	ColorSpacePoint p0, p1, p2, p3;
	//	p0 = _sensor.CoordinateMapper.MapCameraPointToColorSpace(point0);
	//	p1 = _sensor.CoordinateMapper.MapCameraPointToColorSpace(point1);
	//	p2 = _sensor.CoordinateMapper.MapCameraPointToColorSpace(point2);
	//	p3 = _sensor.CoordinateMapper.MapCameraPointToColorSpace(point3);
	//
	//	Canvas.SetLeft(gazePoint0, p0.x - gazePoint0.Width / 2.0);
	//	Canvas.SetTop(gazePoint0, p0.y - gazePoint0.Height / 2.0);
	//	Canvas.SetLeft(gazePoint1, p1.x - gazePoint1.Width / 2.0);
	//	Canvas.SetTop(gazePoint1, p1.y - gazePoint1.Height / 2.0);
	//	Canvas.SetLeft(gazePoint2, p2.x - gazePoint2.Width / 2.0);
	//	Canvas.SetTop(gazePoint2, p2.y - gazePoint2.Height / 2.0);
	//	Canvas.SetLeft(gazePoint3, p3.x - gazePoint3.Width / 2.0);
	//	Canvas.SetTop(gazePoint3, p3.y - gazePoint3.Height / 2.0);
	//
	//	if (gazePoint0.Visibility != Visibility.Visible)
	//		gazePoint0.Visibility = Visibility.Visible;
	//	if (gazePoint1.Visibility != Visibility.Visible)
	//		gazePoint1.Visibility = Visibility.Visible;
	//	if (gazePoint2.Visibility != Visibility.Visible)
	//		gazePoint2.Visibility = Visibility.Visible;
	//	if (gazePoint3.Visibility != Visibility.Visible)
	//		gazePoint3.Visibility = Visibility.Visible;
}

Point2d CalculatePOR(double plane_A, double plane_B, double plane_C, double plane_D, Point3d & c, Point3d & nv, Mat & image){
	// Intersection between Gaze (= c + lambda*Nv) and Display
	Point2d POR;
	// Intersection
	// A(c1+lambda*Nv1)+B(c2+lambda*Nv2)+C(c3+lambda*Nv3)+D=0
	// c -> cornea center, Nv -> visual axis
	double lambda = ((-plane_A * c.x) + (-plane_B * c.y) + (-plane_C * c.z) + (-plane_D)) /
			(plane_A * nv.x + plane_B * nv.y + plane_C * nv.z);
	// Replace lambda in parametrics equations of the gaze line:
	// x = c1 + lambda*Nv1
	// y = c2 + lambda*Nv2
	// z = c3 + lambda*Nv3
	Point3d intersection;
	intersection.x = (double)(c.x + lambda * nv.x);
	intersection.y = (double)(c.y + lambda * nv.y);
	intersection.z = (double)(c.z + lambda * nv.z);

	//	cout << "Intersection: " << intersection << endl;
	//	cout << "Cornea: " << c << "Vector : " << nv << endl;
	// From CameraSpacePoint to ColorSpacePoint in the screen

	Point2d p1 (10, -235), p2 (-218.5, -235), p3 (10, -105), p4 (-218.5, -105);
	vector<Point2d> v_p; v_p.push_back(p1); v_p.push_back(p2); v_p.push_back(p3); v_p.push_back(p4);

	for (int i = 0; i < (int)v_p.size(); i++){
		double x = (v_p[i].x * - 1.0) + 125.0;
		double y = (v_p[i].y) + 300.0;
		//		cout << "[" << x << ", " << y << "]" << " : ";
		Point P (x * 1920 / 458.0, y * 1080 / 260.0);
		//		cout << "P" << i << ": " << P << endl;
		circle(image, P, 5, Scalar(0, 255, 255), -1);
	}

	//	if (intersection.x <= 125.0 && intersection.x >= -333.0 &&
	//			intersection.y <= -40.0 && intersection.y >= -300.0){
	double xvalue = (intersection.x * - 1.0) + 125.0; // [0,0.51]
	double yvalue = (intersection.y) + 300.0; // [0,0.29]
	POR.x = (double)(xvalue * 1920 / 458.0);
	POR.y = (double)(yvalue * 1080 / 260.0);
	//	}
	//	else
	//	{
	//		POR.x = -1;
	//		POR.y = -1;
	//	}

	return POR;
}
