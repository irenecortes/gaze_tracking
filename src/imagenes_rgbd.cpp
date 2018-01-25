
#include "detect_face.h"

#define VIEW 1

pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
#ifdef QHD
Mat depthData(540, 960, CV_32FC3, Scalar(0.0, 0.0, 0.0));
#endif
#ifndef QHD
Mat depthData(1080, 1920, CV_32FC3, Scalar(0.0, 0.0, 0.0));
#endif
		Mat imageRGB;

vector<Point3f> puntos_cara_3d;
vector<Point2i> puntos_cara_2d;

#ifdef QHD
cv::Rect roi_sig (0, 0, 960, 540);
cv::Rect roi_anterior (0, 0, 960, 540);
#endif
#ifndef QHD
cv::Rect roi_sig (0, 0, 1920, 1080);
cv::Rect roi_anterior (0, 0, 1920, 1080);
#endif


int tamano_filtro = 11;

void callback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud_aux;

	pcl::fromROSMsg(*input_cloud, cloud_aux);
	_cloud = cloud_aux.makeShared();

//	depthData = Mat(_cloud->height, _cloud->width, CV_32FC3, Scalar(0.0, 0.0, 0.0));
	//unsigned t0, t1;

	//t0=clock();

	//	if (roi_anterior.x != 0 && roi_anterior.y != 0){
	//		float x_min, x_max, y_min, y_max;
	//		x_min = _cloud->points[roi_anterior.y * _cloud->width + roi_anterior.x].x;
	//		y_min = _cloud->points[roi_anterior.y * _cloud->width + roi_anterior.x].y;
	//
	//		x_max = _cloud->points[(roi_anterior.y + roi_anterior.height) * _cloud->width + roi_anterior.x + roi_anterior.width].x;
	//		y_max = _cloud->points[(roi_anterior.y + roi_anterior.height) * _cloud->width + roi_anterior.x + roi_anterior.width].y;
	//		//	Mat pointcloud_image (_cloud->height, _cloud->width, CV_32FC3, _cloud->points);
	//		//	imshow("PCL_IM", pointcloud_image);
	//
	//		cout << "min: " << x_min << ", " << y_min << ". max: " << x_max << ", " << y_max << endl;
	//		cout << _cloud->width << ", " << _cloud->height << endl;
	//		// Create the filtering object
	//		pcl::PassThrough<pcl::PointXYZRGB> pass;
	//		pass.setInputCloud (_cloud);
	//		pass.setFilterFieldName ("x");
	//		pass.setFilterLimits (x_min, x_max);
	//		pass.filter (*_cloud_filtered);
	//
	//		pass.setInputCloud (_cloud_filtered);
	//
	//		pass.setFilterFieldName ("y");
	//		pass.setFilterLimits (y_min, y_max);
	//		pass.filter (*_cloud);
	//
	//		cout << roi_anterior.width << ", " << roi_anterior.height << endl;
	//		cout << _cloud->width << ", " << _cloud->height << endl;
	//	}

	for (int i = 0, v = 0; v < _cloud->height; v++){
		for (int u = 0; u < _cloud->width; u++, i++){

			//	for (int i = roi_anterior.y * _cloud->width + roi_anterior.x, v = 0; v < roi_anterior.height; v++, i += (_cloud->width - roi_anterior.width)){
			//		for (int u = 0; u < roi_anterior.width; u++, i++){
			// Obtiene la información de profundidad de la nube de puntos
			if (!isnan(_cloud->points[i].x)/* && cloud_aux.points[i].z < max*/)
				depthData.at<Vec3f>(v,u)[0] = _cloud->points[i].x * 1000;
			if (!isnan(_cloud->points[i].y)/* && cloud_aux.points[i].z < max*/)
				depthData.at<Vec3f>(v,u)[1] = _cloud->points[i].y * 1000;
			if (!isnan(_cloud->points[i].z)/* && cloud_aux.points[i].z < max*/)
				depthData.at<Vec3f>(v,u)[2] = _cloud->points[i].z * 1000;
		}
	}


	cv_bridge::CvImagePtr img;
	img = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::BGR8); // container of type sensor_msgs/Image
	imageRGB = img->image;

	//t1 = clock();

	//double time = (double(t1-t0)/CLOCKS_PER_SEC);
	//cout << "Execution Time: " << time << endl;
}

int main (int argc, char** argv) {

	struct Face driver;

	Point2d right_pupil, left_pupil;
	//	vector<Point3f> puntos_cara_3d;
	//	vector<Point2i> puntos_cara_2d;
	vector<Point3f> puntos_ojos_3D;
	vector<Point2i> puntos_ojos_2D;

	int num_images = 0;
	std::vector<cv::Point3f> Points_ideal, Points_real;
	Inicializar3D_Ideal(Points_ideal);

	vector<Mat> rvecs, tvecs;
	vector<Point2d> v_right_pupil, v_left_pupil;

	namedWindow("Cara detectada", CV_WINDOW_NORMAL);

	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	dlib::shape_predictor pose_model;
	dlib::deserialize("/home/ivvi2018/catkin_ws/src/face_detection/shape_predictor_68_face_landmarks.dat") >> pose_model;
	//	dlib::image_window win;

	try{
		//	 Initializa ROS
		ros::init (argc, argv, "imagenes_rgbd");
		ros::NodeHandle nh;
		std_msgs::String msg;

		//	 Crear subscriptores para la nube de puntos y la imagen de color

#ifdef QHD
		message_filters::Subscriber<sensor_msgs::Image> sub_color(nh, "/kinect2/qhd/image_color_rect", 1);
		message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points(nh, "/kinect2/qhd/points", 1);
#endif
#ifndef QHD
		message_filters::Subscriber<sensor_msgs::Image> sub_color(nh, "/kinect2/hd/image_color_rect", 1);
		message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points(nh, "/kinect2/hd/points", 1);
#endif

		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_color, sub_points);
		sync.registerCallback(boost::bind(&callback, _1, _2));

		ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/face/pose", 1);
		//		ros::Publisher pub_img = nh.advertise<sensor_msgs::Image>("/face/image_ROI", 1);
		//		ros::Publisher pub_points3d = nh.advertise<tfm_msgs::Vector_Points>("/face/points3d", 1);
		//		ros::Publisher pub_points2d = nh.advertise<tfm_msgs::Vector_Points>("/face/points2d", 1);
		ros::Publisher pub_points3d_eyes = nh.advertise<tfm_msgs::Vector_Points>("/eyes/points3d", 1);
		ros::Publisher pub_points2d_eyes = nh.advertise<tfm_msgs::Vector_Points>("/eyes/points2d", 1);

		std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
		double fps = 0;
		size_t frameCount = 0;
		std::ostringstream oss;

		ros::Rate r(50);
		start = std::chrono::high_resolution_clock::now();
		while (nh.ok()) {
			//		 Spin
			ros::spinOnce();
#ifdef QHD
			Mat detected_face_im (540, 960, CV_8UC3, Scalar(0, 0, 0));
#endif
#ifndef QHD
			Mat detected_face_im (1080, 1920, CV_8UC3, Scalar(0, 0, 0));
#endif

			if (!imageRGB.empty() && !depthData.empty()){
				Mat depthData_small(1, 1, CV_32FC3), imageRGB_small(1, 1, CV_8UC3);

				imageRGB_small = imageRGB(roi_anterior);
				depthData_small = depthData(roi_anterior);

				dlib::cv_image<dlib::bgr_pixel> cimg(imageRGB_small);
				std::vector<dlib::rectangle> faces = detector(cimg);
				std::vector<dlib::full_object_detection> shapes;
				for (unsigned long i = 0; i < faces.size(); ++i){
					shapes.push_back(pose_model(cimg, faces[i]));

					roi_sig = Rect(roi_anterior.x + faces[i].left() - faces[i].width() / 2 , roi_anterior.y + faces[i].top() - faces[i].height() / 2.0, 2*faces[i].width(), 2*faces[i].height());

					if(roi_sig.x < 0){
						roi_sig.width += roi_sig.x;
						roi_sig.x = 0;
					}
					if(roi_sig.y < 0){
						roi_sig.height += roi_sig.y;
						roi_sig.y = 0;
					}
				}

				if(faces.empty()){
					driver.presencia = 0;
					continue;
				} else driver.presencia = 1;

				//puntos_cara_3d.clear();
				puntos_cara_2d.clear();
				puntos_ojos_3D.clear();
				puntos_ojos_2D.clear();
				ObtienePuntos(puntos_cara_3d, puntos_cara_2d, puntos_ojos_3D, puntos_ojos_2D, shapes, roi_anterior, driver, depthData_small);

				cv::Mat transf (3, 4, CV_64FC1), inliers;
				Inicializar3D_Real(Points_real);
				estimateAffine3D(Points_ideal, Points_real, transf, inliers, 1, 0.99);

				cv::Mat rotation (3, 3, CV_64FC1, Scalar(0.0)), tvec(3, 1, CV_64FC1, Scalar(0.0)), rvec(3, 1, CV_64FC1, Scalar(0.0));
				ObtieneTransfVec(transf, rotation, rvec, tvec);
				Mat rvec_filtered(3, 1, CV_64FC1, Scalar(0.0)), tvec_filtered(3, 1, CV_64FC1, Scalar(0.0));
				filter_pose(rvec, tvec, rvecs, tvecs, rvec_filtered, tvec_filtered);
				ObtieneTransfMat(transf, rotation, rvec_filtered, tvec_filtered);

				vector<Point3d> puntos_ojos_centrados;
				vector<Point2d> puntos2d_ojos_centrados;

				puntos_ojos_centrados.push_back(Point3d(tvec_filtered.at<double>(0), tvec_filtered.at<double>(1), tvec_filtered.at<double>(2)));
				puntos_ojos_centrados.push_back(Points_real[N_RIGHT_EYER]);
				puntos_ojos_centrados.push_back(Points_real[N_RIGHT_EYEL]);
				puntos_ojos_centrados.push_back(Points_real[N_LEFT_EYER]);
				puntos_ojos_centrados.push_back(Points_real[N_LEFT_EYEL]);
				projectPoints(puntos_ojos_centrados, cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, puntos2d_ojos_centrados, cv::noArray(), 0);

				Point2d right_pupil_filtered, left_pupil_filtered;
				PupilDetection(puntos_ojos_2D, right_pupil, left_pupil, imageRGB);
				filter_pupils(right_pupil, left_pupil, v_right_pupil, v_left_pupil, right_pupil_filtered, left_pupil_filtered);
				puntos_cara_2d.push_back(right_pupil_filtered);
				puntos_ojos_2D.push_back(right_pupil_filtered);
				Point3f right_pupil_w = depthData.at<Vec3f>(right_pupil_filtered.y,right_pupil_filtered.x);
				if (puntos_cara_3d.size() < RIGHT_PUPIL){
					puntos_cara_3d.push_back(right_pupil_w);
				} else {
					if (right_pupil_w != Point3f(0.0, 0.0, 0.0)){
						puntos_cara_3d[RIGHT_PUPIL] = right_pupil_w;
					}
					puntos_ojos_3D.push_back(puntos_cara_3d[RIGHT_PUPIL]);
				}

				puntos_cara_2d.push_back(left_pupil_filtered);
				puntos_ojos_2D.push_back(left_pupil_filtered);
				Point3f left_pupil_w = depthData.at<Vec3f>(left_pupil_filtered.y,left_pupil_filtered.x);
				if (puntos_cara_3d.size() < LEFT_PUPIL){
					puntos_cara_3d.push_back(left_pupil_w);
				} else {
					if (left_pupil_w != Point3f(0.0, 0.0, 0.0)){
						puntos_cara_3d[LEFT_PUPIL] = left_pupil_w;
					}
					puntos_ojos_3D.push_back(puntos_cara_3d[LEFT_PUPIL]);
				}

				//				puntos_cara_3d.push_back(left_pupil_w);
				//				puntos_ojos_3D.push_back(left_pupil_w);

				//				cout << puntos_cara_3d.size() << endl;
				imageRGB_small.copyTo(detected_face_im(roi_anterior), imageRGB_small);
				for (int i = 0; i < (int)puntos2d_ojos_centrados.size(); i++){
					cv::circle(detected_face_im, puntos2d_ojos_centrados[i], 3, cv::Scalar(0,255,255,0), -1);
				}
				if (VIEW){
					DibujarRostro(detected_face_im, puntos_cara_2d, Scalar(0, 0, 255));
					proyectarRostro(detected_face_im, Points_real, Points_ideal, transf);
					DibujarEjes(detected_face_im, transf);
					imshow("Cara detectada", detected_face_im);
					waitKey(1);
				}

				//PUBLICAR
				geometry_msgs::PoseStamped pose_msg;
				pose_msg.pose.position.x = tvec_filtered.at<double>(0);
				pose_msg.pose.position.y = tvec_filtered.at<double>(1);
				pose_msg.pose.position.z = tvec_filtered.at<double>(2);

				tf::Quaternion q = tf::createQuaternionFromRPY(rvec_filtered.at<double>(0), rvec_filtered.at<double>(1), rvec_filtered.at<double>(2));
				pose_msg.pose.orientation.x = q.getX();
				pose_msg.pose.orientation.y = q.getY();
				pose_msg.pose.orientation.z = q.getZ();
				pose_msg.pose.orientation.w = q.getW();

				//				tfm_msgs::Vector_Points points3_msg;
				//				tfm_msgs::Vector_Points points2_msg;
				tfm_msgs::Vector_Points points3_eyes_msg;
				tfm_msgs::Vector_Points points2_eyes_msg;

				//				for (int i = 0; i < (int)puntos_cara_2d.size(); i++){
				//					geometry_msgs::Point punto_cara;
				//					punto_cara.x = puntos_cara_3d[i].x;
				//					punto_cara.y = puntos_cara_3d[i].y;
				//					punto_cara.z = puntos_cara_3d[i].z;
				//					points3_msg.puntos.push_back(punto_cara);
				//					punto_cara.x = puntos_cara_2d[i].x + roi_anterior.x;
				//					punto_cara.y = puntos_cara_2d[i].y + roi_anterior.y;
				//					punto_cara.z = 0.0;
				//					points2_msg.puntos.push_back(punto_cara);
				//				}

				for (int i = 0; i < (int)puntos_ojos_2D.size(); i++){
					geometry_msgs::Point punto_ojos;
					punto_ojos.x = puntos_ojos_3D[i].x;
					punto_ojos.y = puntos_ojos_3D[i].y;
					punto_ojos.z = puntos_ojos_3D[i].z;
					points3_eyes_msg.puntos.push_back(punto_ojos);
					punto_ojos.x = puntos_ojos_2D[i].x; // + roi_anterior.x;
					punto_ojos.y = puntos_ojos_2D[i].y; // + roi_anterior.y;
					punto_ojos.z = 0.0;
					points2_eyes_msg.puntos.push_back(punto_ojos);
				}

				//				std_msgs::Int32 n_puntos;
				//				n_puntos.data = puntos_cara_2d.size();
				//				points3_msg.num_puntos = n_puntos;
				//				points2_msg.num_puntos = n_puntos;

				std_msgs::Int32 n_puntos_ojos;
				n_puntos_ojos.data = puntos_ojos_2D.size();
				points3_eyes_msg.num_puntos = n_puntos_ojos;
				points2_eyes_msg.num_puntos = n_puntos_ojos;

				//				points3_msg.header.stamp = ros::Time::now();
				//				points2_msg.header.stamp = ros::Time::now();
				points3_eyes_msg.header.stamp = ros::Time::now();
				points2_eyes_msg.header.stamp = ros::Time::now();
				pose_msg.header.stamp = ros::Time::now();

				pub_pose.publish(pose_msg);
				//				pub_points3d.publish(points3_msg);
				//				pub_points2d.publish(points2_msg);
				pub_points3d_eyes.publish(points3_eyes_msg);
				pub_points2d_eyes.publish(points2_eyes_msg);

				roi_anterior = roi_sig;

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
			}
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

void DibujarRostro(cv::Mat &image, vector<Point2i> cara, Scalar color){
	//	cout << "DibujarRostro " << cara.size() << endl;
	// Around Chin. Ear to Ear
	//	for (unsigned long i = 1; i <= 16; ++i)
	//		cv::line(image, Point(shape.part(i).x(), shape.part(i).y()), Point(shape.part(i-1).x(), shape.part(i-1).y()), cv::Scalar(255,255,0,0), 1, 8, 0);

	if (cara.size() > 48){
		// Line on top of nose
		cv::line(image, Point(cara[14].x, cara[14].y), Point(cara[13].x, cara[13].y), color, 1, 8, 0);

		// left eyebrow
		for (unsigned long i = 4; i <= 7; ++i)
			cv::line(image, Point(cara[i].x, cara[i].y), Point(cara[i-1].x, cara[i-1].y), color, 1, 8, 0);
		// Right eyebrow
		for (unsigned long i = 9; i <= 12; ++i)
			cv::line(image, Point(cara[i].x, cara[i].y), Point(cara[i-1].x, cara[i-1].y), color, 1, 8, 0);
		// Bottom part of the nose
		for (unsigned long i = 16; i <= 17; ++i)
			cv::line(image, Point(cara[i].x, cara[i].y), Point(cara[i-1].x, cara[i-1].y), color, 1, 8, 0);
		// Line from the nose to the bottom part above
		//	cv::line(image, Point(shape.part(30).x(), shape.part(30).y()), Point(shape.part(35).x(), shape.part(35).y()), cv::Scalar(255,255,0,0), 1, 8, 0);

		// Left eye
		for (unsigned long i = 19; i <= 23; ++i)
			cv::line(image, Point(cara[i].x, cara[i].y), Point(cara[i-1].x, cara[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(cara[18].x, cara[18].y), Point(cara[23].x, cara[23].y), color, 1, 8, 0);

		// Right eye
		for (unsigned long i = 25; i <= 29; ++i)
			cv::line(image, Point(cara[i].x, cara[i].y), Point(cara[i-1].x, cara[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(cara[24].x, cara[24].y), Point(cara[29].x, cara[29].y), color, 1, 8, 0);

		// Lips outer part
		for (unsigned long i = 31; i <= 41; ++i)
			cv::line(image, Point(cara[i].x, cara[i].y), Point(cara[i-1].x, cara[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(cara[30].x, cara[30].y), Point(cara[41].x, cara[41].y), color, 1, 8, 0);

		// Lips inside part
		for (unsigned long i = 43; i <= 47; ++i)
			cv::line(image, Point(cara[i].x, cara[i].y), Point(cara[i-1].x, cara[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(cara[42].x, cara[42].y), Point(cara[47].x, cara[47].y), color, 1, 8, 0);


		circle(image, Point(cara[RIGHT_PUPIL].x, cara[RIGHT_PUPIL].y), 1, cv::Scalar(255, 0, 255), -1);
		circle(image, Point(cara[LEFT_PUPIL].x, cara[LEFT_PUPIL].y), 1, cv::Scalar(255, 0, 255), -1);
	}
}

void proyectarRostro(Mat & image, vector<Point3f> & puntos_cara, vector<Point3f> & vector_p_ideales, Mat & tran){
	cv::Point3f Pto3D_camera;
	std::vector<cv::Point3f> PointsRostro;
	std::vector<cv::Point3f> PointsIdeales;
	Mat Pto3D_H;

	//Se definen los puntos de los ejes tridimensionalmente y se transforman para que el origen de coordenadas
	//aparezca entre los ojos de la cara detectada
	for (int i = 0; i < (int)vector_p_ideales.size(); i++){
		Pto3D_H = (Mat_<double>(4,1) << vector_p_ideales[i].x, vector_p_ideales[i].y, vector_p_ideales[i].z, 1);
		Pto3D_H = tran*Pto3D_H;
		Pto3D_camera.x = Pto3D_H.at<double>(0);// / 1000.0;
		Pto3D_camera.y = Pto3D_H.at<double>(1);// / 1000.0;
		Pto3D_camera.z = Pto3D_H.at<double>(2);// / 1000.0;
		PointsIdeales.push_back(Pto3D_camera);
	}

	//Se proyectan los puntos tridimensionales en la imagen
	std::vector<cv::Point2f>  Pto2DRostro, Pto2DIdeales;
	projectPoints(PointsIdeales, /*cameraRot*/ cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, Pto2DIdeales, cv::noArray(), 0);
	projectPoints(puntos_cara, /*cameraRot*/ cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, Pto2DRostro, cv::noArray(), 0);

	//	for (int i = 0; i < (int)Pto2DIdeales.size() - 1; i++){
	//		cv::line(image, Pto2DIdeales[i],Pto2DIdeales[i + 1], cv::Scalar(255,255,0,0), 1, 8, 0);
	//	}

	for (int i = 0; i < (int)Pto2DRostro.size() - 1; i++){
		cv::line(image, Pto2DRostro[i],Pto2DRostro[i + 1], cv::Scalar(0,255,255,0), 1, 8, 0);
	}
}

//Almacena en Points3D y Points2D los puntos detectados de la cara
//En Points2D se almacena la información en píxeles y en Points3D la información en milimetros
void Inicializar3D_Real(std::vector<cv::Point3f> & Points3D){
	Points3D.clear();
	Points3D.push_back(puntos_cara_3d[RIGHT_EYER]);
	Points3D.push_back(puntos_cara_3d[RIGHT_EYEL]);
	Points3D.push_back(puntos_cara_3d[LEFT_EYER]);
	Points3D.push_back(puntos_cara_3d[LEFT_EYEL]);
	Points3D.push_back(puntos_cara_3d[NOSE]);
	Points3D.push_back(puntos_cara_3d[RIGHT_NOSE]);
	Points3D.push_back(puntos_cara_3d[CTR_NOSE]);
	Points3D.push_back(puntos_cara_3d[LEFT_NOSE]);
	Points3D.push_back(puntos_cara_3d[STOMMION]);
	Points3D.push_back(puntos_cara_3d[RIGHT_LIP_1]);
	Points3D.push_back(puntos_cara_3d[LEFT_LIP_3]);
	Points3D.push_back(puntos_cara_3d[MENTON]);
}

void Inicializar2D_Real(std::vector<cv::Point2i> & Points2D){
	Points2D.clear();
	Points2D.push_back(puntos_cara_2d[RIGHT_EYER]);
	Points2D.push_back(puntos_cara_2d[RIGHT_EYEL]);
	Points2D.push_back(puntos_cara_2d[LEFT_EYER]);
	Points2D.push_back(puntos_cara_2d[LEFT_EYEL]);
	Points2D.push_back(puntos_cara_2d[NOSE]);
	Points2D.push_back(puntos_cara_2d[RIGHT_NOSE]);
	Points2D.push_back(puntos_cara_2d[CTR_NOSE]);
	Points2D.push_back(puntos_cara_2d[LEFT_NOSE]);
	Points2D.push_back(puntos_cara_2d[STOMMION]);
	Points2D.push_back(puntos_cara_2d[RIGHT_LIP_1]);
	Points2D.push_back(puntos_cara_2d[LEFT_LIP_3]);
	Points2D.push_back(puntos_cara_2d[MENTON]);
}

void Inicializar3D_Ideal(std::vector<cv::Point3f> & Points3D){
	Points3D.clear();
	Points3D.push_back(P3D_RIGHT_EYER);
	Points3D.push_back(P3D_RIGHT_EYEL);
	Points3D.push_back(P3D_LEFT_EYER);
	Points3D.push_back(P3D_LEFT_EYEL);
	Points3D.push_back(P3D_NOSE);
	Points3D.push_back(P3D_NOSEBR);
	Points3D.push_back(P3D_NOSEBC);
	Points3D.push_back(P3D_NOSEBL);
	Points3D.push_back(P3D_STOMMION);
	Points3D.push_back(P3D_LIPR);
	Points3D.push_back(P3D_LIPL);
	Points3D.push_back(P3D_MENTON);
}


//Almacena en Points3D y Points2D los puntos detectados de la cara
//En Points2D se almacena la información en píxeles y en Points3D la información en milimetros
void ObtienePuntos(std::vector<cv::Point3f> &Points3D, std::vector<cv::Point2i> &Points2D,
		vector<cv::Point3f> &PointsEyes3D, vector<cv::Point2i> &PointsEyes2D,
		std::vector<dlib::full_object_detection> & shapes, Rect & roi, Face & driver, Mat & depth){

	vector<int> puntos_cara;

	puntos_cara.push_back(ORIGINAL_RIGHT_EAR);
	puntos_cara.push_back(ORIGINAL_MENTON);
	puntos_cara.push_back(ORIGINAL_LEFT_EAR);
	puntos_cara.push_back(ORIGINAL_RIGHT_BROW_1);
	puntos_cara.push_back(ORIGINAL_RIGHT_BROW_2);
	puntos_cara.push_back(ORIGINAL_RIGHT_BROW_3);
	puntos_cara.push_back(ORIGINAL_RIGHT_BROW_4);
	puntos_cara.push_back(ORIGINAL_RIGHT_BROW_5);
	puntos_cara.push_back(ORIGINAL_LEFT_BROW_1);
	puntos_cara.push_back(ORIGINAL_LEFT_BROW_2);
	puntos_cara.push_back(ORIGINAL_LEFT_BROW_3);
	puntos_cara.push_back(ORIGINAL_LEFT_BROW_4);
	puntos_cara.push_back(ORIGINAL_LEFT_BROW_5);
	puntos_cara.push_back(ORIGINAL_CTRE_EYES);
	puntos_cara.push_back(ORIGINAL_NOSE);
	puntos_cara.push_back(ORIGINAL_RIGHT_NOSE);
	puntos_cara.push_back(ORIGINAL_CTR_NOSE);
	puntos_cara.push_back(ORIGINAL_LEFT_NOSE);
	puntos_cara.push_back(ORIGINAL_RIGHT_EYER);
	puntos_cara.push_back(ORIGINAL_RIGHT_EYE_TOP_R);
	puntos_cara.push_back(ORIGINAL_RIGHT_EYE_TOP_L);
	puntos_cara.push_back(ORIGINAL_RIGHT_EYEL);
	puntos_cara.push_back(ORIGINAL_RIGHT_EYE_BOTTOM_L);
	puntos_cara.push_back(ORIGINAL_RIGHT_EYE_BOTTOM_R);
	puntos_cara.push_back(ORIGINAL_LEFT_EYER);
	puntos_cara.push_back(ORIGINAL_LEFT_EYE_TOP_R);
	puntos_cara.push_back(ORIGINAL_LEFT_EYE_TOP_L);
	puntos_cara.push_back(ORIGINAL_LEFT_EYEL);
	puntos_cara.push_back(ORIGINAL_LEFT_EYE_BOTTOM_L);
	puntos_cara.push_back(ORIGINAL_LEFT_EYE_BOTTOM_R);
	puntos_cara.push_back(ORIGINAL_RIGHT_LIP_1);
	puntos_cara.push_back(ORIGINAL_RIGHT_LIP_2);
	puntos_cara.push_back(ORIGINAL_RIGHT_LIP_3);
	puntos_cara.push_back(ORIGINAL_STOMMION);
	puntos_cara.push_back(ORIGINAL_LEFT_LIP_1);
	puntos_cara.push_back(ORIGINAL_LEFT_LIP_2);
	puntos_cara.push_back(ORIGINAL_LEFT_LIP_3);
	puntos_cara.push_back(ORIGINAL_BOTTOM_LIP_1);
	puntos_cara.push_back(ORIGINAL_BOTTOM_LIP_2);
	puntos_cara.push_back(ORIGINAL_BOTTOM_LIP_3);
	puntos_cara.push_back(ORIGINAL_BOTTOM_LIP_4);
	puntos_cara.push_back(ORIGINAL_BOTTOM_LIP_5);
	puntos_cara.push_back(ORIGINAL_INNER_LIP_1);
	puntos_cara.push_back(ORIGINAL_INNER_LIP_2);
	puntos_cara.push_back(ORIGINAL_INNER_LIP_3);
	puntos_cara.push_back(ORIGINAL_INNER_LIP_4);
	puntos_cara.push_back(ORIGINAL_INNER_LIP_5);
	puntos_cara.push_back(ORIGINAL_INNER_LIP_6);

	cv::Point3f Pto3D;
	cv::Point2i Pto_Im;

	if (Points3D.size() == 0){
		for (int i = 0; i < puntos_cara.size(); i++){
			Pto_Im.x = shapes[0].part(puntos_cara[i]).x();
			Pto_Im.y = shapes[0].part(puntos_cara[i]).y();
			Pto3D = depth.at<Vec3f>(Pto_Im.y,Pto_Im.x);
			Points2D.push_back(Point2i(Pto_Im.x + roi.x, Pto_Im.y + roi.y));
			Points3D.push_back(Pto3D);
		}
	} else {
		for (int i = 0; i < puntos_cara.size(); i++){
			Pto_Im.x = shapes[0].part(puntos_cara[i]).x();
			Pto_Im.y = shapes[0].part(puntos_cara[i]).y();
			Pto3D = depth.at<Vec3f>(Pto_Im.y,Pto_Im.x);
			Points2D.push_back(Point2i(Pto_Im.x + roi.x, Pto_Im.y + roi.y));
			if (Pto3D != Point3f(0.0, 0.0, 0.0)){
				Points3D[i] = Pto3D;
			}
		}
	}

	for (int i = RIGHT_EYER; i <= LEFT_EYE_BOTTOM_R; i++){
		PointsEyes2D.push_back(Points2D[i]);
		PointsEyes3D.push_back(Points3D[i]);
	}
}

//Obtiene la matriz de rotación y los vectores de rotación y traslación. Convierte la matriz de transformación en una 4x4 con la que se puede operar
void ObtieneTransfVec(Mat & transf, Mat & rotation, Mat & rvec, Mat & tvec){
	rotation.at<double>(0,0) = transf.at<double>(0,0);
	rotation.at<double>(0,1) = transf.at<double>(0,1);
	rotation.at<double>(0,2) = transf.at<double>(0,2);
	rotation.at<double>(1,0) = transf.at<double>(1,0);
	rotation.at<double>(1,1) = transf.at<double>(1,1);
	rotation.at<double>(1,2) = transf.at<double>(1,2);
	rotation.at<double>(2,0) = transf.at<double>(2,0);
	rotation.at<double>(2,1) = transf.at<double>(2,1);
	rotation.at<double>(2,2) = transf.at<double>(2,2);
	//	cout << "R: " << rotation << endl;

	tvec.at<double>(0) = transf.at<double>(0,3);
	tvec.at<double>(1) = transf.at<double>(1,3);
	tvec.at<double>(2) = transf.at<double>(2,3);
	//	cout << "tvec: " << tvec << endl;

	Rodrigues(rotation,rvec);
	//	cout << "Rvec: " << rvec << endl;

	cv::Mat row = (Mat_<double>(1,4) << 0.0, 0.0, 0.0, 1.0);
	transf.push_back(row);
}

void ObtieneTransfMat(Mat & transf, Mat & rotation, Mat & rvec, Mat & tvec){
	Rodrigues(rvec,rotation);

	transf.at<double>(0,0) = rotation.at<double>(0,0);
	transf.at<double>(0,1) = rotation.at<double>(0,1);
	transf.at<double>(0,2) = rotation.at<double>(0,2);
	transf.at<double>(1,0) = rotation.at<double>(1,0);
	transf.at<double>(1,1) = rotation.at<double>(1,1);
	transf.at<double>(1,2) = rotation.at<double>(1,2);
	transf.at<double>(2,0) = rotation.at<double>(2,0);
	transf.at<double>(2,1) = rotation.at<double>(2,1);
	transf.at<double>(2,2) = rotation.at<double>(2,2);

	transf.at<double>(0,3) = tvec.at<double>(0);
	transf.at<double>(1,3) = tvec.at<double>(1);
	transf.at<double>(2,3) = tvec.at<double>(2);
}

void filter_pose(Mat & rvec, Mat & tvec, vector<Mat> & rvecs, vector<Mat> & tvecs, Mat & rvec_f, Mat & tvec_f){
	if ((int)rvecs.size() >= tamano_filtro){
		for (int i = 0; i < rvecs.size() - 1; i++){
			rvecs[i] = rvecs[i + 1];
		}
		rvecs.pop_back();
		rvecs.push_back(rvec);

		for (int i = 0; i < tvecs.size() - 1; i++){
			tvecs[i] = tvecs[i + 1];
		}
		tvecs.pop_back();
		tvecs.push_back(tvec);
	} else {
		rvecs.push_back(rvec);
		tvecs.push_back(tvec);
	}

	vector<double> roll, pitch, yaw, x, y, z;
	for (int i = 0; i < (int)rvecs.size(); i++){
		roll.push_back(rvecs[i].at<double>(0));
		pitch.push_back(rvecs[i].at<double>(1));
		yaw.push_back(rvecs[i].at<double>(2));
	}
	for (int i = 0; i < (int)tvecs.size(); i++){
		x.push_back(tvecs[i].at<double>(0));
		y.push_back(tvecs[i].at<double>(1));
		z.push_back(tvecs[i].at<double>(2));
		//		cout << tvecs[i].at<double>(0) << ", ";
	}

	sort(roll.begin(), roll.end());
	sort(pitch.begin(), pitch.end());
	sort(yaw.begin(), yaw.end());
	sort(x.begin(), x.end());
	sort(y.begin(), y.end());
	sort(z.begin(), z.end());

	rvec_f.at<double>(0) = roll[roll.size() / 2];
	rvec_f.at<double>(1) = pitch[pitch.size() / 2];
	rvec_f.at<double>(2) = yaw[yaw.size() / 2];

	tvec_f.at<double>(0) = x[x.size() / 2];
	tvec_f.at<double>(1) = y[y.size() / 2];
	tvec_f.at<double>(2) = z[z.size() / 2];
	//	cout << rvecs.size() << ", " << tvecs.size() << endl;
	//	cout << x.size() << ", " << y.size() << ", " << z.size() << ", " << roll.size() << ", " << pitch.size() << ", " << yaw.size() << endl;
	//	waitKey(0);
}

void filter_pupils(Point2d & right_pupil, Point2d & left_pupil, vector<Point2d> & v_right_pupil, vector<Point2d> & v_left_pupil, Point2d & right_pupil_f, Point2d & left_pupil_f){
	if ((int)v_right_pupil.size() >= tamano_filtro){
		for (int i = 0; i < v_right_pupil.size() - 1; i++){
			v_right_pupil[i] = v_right_pupil[i + 1];
		}
		v_right_pupil.pop_back();
		v_right_pupil.push_back(right_pupil);

		for (int i = 0; i < v_left_pupil.size() - 1; i++){
			v_left_pupil[i] = v_left_pupil[i + 1];
		}
		v_left_pupil.pop_back();
		v_left_pupil.push_back(left_pupil);
	} else {
		v_right_pupil.push_back(right_pupil);
		v_left_pupil.push_back(left_pupil);
	}

	vector<double> x_i, y_i, x_d, y_d;
	for (int i = 0; i < (int)v_right_pupil.size(); i++){
		x_d.push_back(v_right_pupil[i].x);
		y_d.push_back(v_right_pupil[i].y);
	}
	for (int i = 0; i < (int)v_left_pupil.size(); i++){
		x_i.push_back(v_left_pupil[i].x);
		y_i.push_back(v_left_pupil[i].y);
	}

	sort(x_i.begin(), x_i.end());
	sort(y_i.begin(), y_i.end());
	sort(x_d.begin(), x_d.end());
	sort(y_d.begin(), y_d.end());

	right_pupil_f = Point2d(x_d[x_d.size() / 2], y_d[y_d.size() / 2]);
	left_pupil_f = Point2d(x_i[x_i.size() / 2], y_i[y_i.size() / 2]);
}
void DibujarEjes(cv::Mat &cam, cv::Mat & tran) {
	cv::Point3f Pto3D_camera;
	std::vector<cv::Point3f> PointsEjes;
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


	//Se proyectan los puntos tridimensionales en la imagen
	std::vector<cv::Point2f>  Pto2DEjes;
	projectPoints(PointsEjes, /*cameraRot*/ cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, Pto2DEjes, cv::noArray(), 0);

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

// Pupil detection method
void PupilDetection(vector<Point2i> & puntos_ojos, Point2d & right_pupil, Point2d & left_pupil, Mat & imagen){
	//	cout << puntos_ojos.size() << endl;

	if (puntos_ojos.size() > 10){
		Point2i point1 = puntos_ojos[EYES_LEFT_EYEL] ; //lefteyeOutercorner);

		if (point1.x <= 0)
		{
			left_pupil.x = -1;
			left_pupil.y = -1;
		}
		else
		{
			Point2i point2 = puntos_ojos[EYES_LEFT_EYER] ;
			Point2i point3 = (puntos_ojos[EYES_LEFT_EYE_TOP_L]  + puntos_ojos[EYES_LEFT_EYE_TOP_R] )/ 2.0;
			Point2i point4 = (puntos_ojos[EYES_LEFT_EYE_BOTTOM_L]  + puntos_ojos[EYES_LEFT_EYE_BOTTOM_R] ) / 2.0;

			double eyeWidth = abs(point1.x - point2.x);
			double eyeHeight = abs(point4.y - point3.y);

			// LEFT EYE ROI
			Point2i ojo_i_medio = (puntos_ojos[EYES_LEFT_EYER]  + puntos_ojos[EYES_LEFT_EYEL]  ) / 2.0;
			int x = (int)(ojo_i_medio.x - ((eyeWidth / 2)));// - 0.2 * eyeWidth));
			int y = (int)(ojo_i_medio.y - ((eyeHeight / 2)));// - 0.35 * eyeHeight));
			int width = (int)((eyeWidth)); // - 0.30 * eyeWidth);
			int height = (int)((eyeHeight)); // - 0.2 * eyeHeight);

			//		cout << "LEFT ROI: " << x << ", " << y << ", " << width << ", " << height << endl;

			if (x >= 0 && y >= 0 && width > 0 && height > 0){
				Mat ROI = imagen(Rect(x, y, width, height));
				//			imshow("Image_izq", ROI);
				//			waitKey(1);

				// Gray scale
				Mat grayImage;
				cvtColor(ROI, grayImage, CV_RGB2GRAY);

				// Normalizes brightness and increases contrast
				equalizeHist(grayImage, grayImage);

				// Black and white (inverted)

				Mat blackAndWhiteImage;
				threshold(grayImage, blackAndWhiteImage, 60, 255, THRESH_BINARY_INV); // este parametro se puede aprender
				// Bigger x4
				Mat resizeImage;
				resize(blackAndWhiteImage, resizeImage, Size(), 4.0, 4.0);

				// Erode filter
				Mat erodeImage;
				erode(resizeImage, erodeImage, Mat());
				//				Image<Gray, Byte> erodeImage = resizeImage.Erode(4);

				//				imshow("ErodeImage", erodeImage);
				//				waitKey(1);

				Moments momentos = moments(erodeImage);
				Point2d pupil (momentos.m10 / momentos.m00, momentos.m01 / momentos.m00);

				Point2d pupilInColorSpace;
				pupilInColorSpace.x = x + (double)(pupil.x / 4.0);
				pupilInColorSpace.y = y + (double)(pupil.y / 4.0);

				left_pupil = pupilInColorSpace;
				//				cout << "Pupila izquierda detectada: " << left_pupil << endl;
			}
		}

		// Head turned to left -> right eye is used.

		point1 = puntos_ojos[EYES_RIGHT_EYEL] ; //(righteyeInnercorner);
		//Console.WriteLine("1: " + (int)point1.x);
		//Console.WriteLine("2: " + (int)point1.y);

		if (point1.x <= 0)
		{
			right_pupil.x = -1;
			right_pupil.y = -1;
			//		ojo_d.presencia = 0;
		}
		else
		{
			//		ojo_d.presencia = 1;

			Point2i point2 = puntos_ojos[EYES_RIGHT_EYER] ;
			Point2i point3 = (puntos_ojos[EYES_RIGHT_EYE_TOP_L]  + puntos_ojos[EYES_RIGHT_EYE_TOP_R] ) / 2.0;
			Point2i point4 = (puntos_ojos[EYES_RIGHT_EYE_BOTTOM_L]  + puntos_ojos[EYES_RIGHT_EYE_BOTTOM_R] ) / 2.0;

			double eyeWidth = abs(point1.x - point2.x);
			double eyeHeight = abs(point4.y - point3.y);

			// RIGHT EYE
			Point2i ojo_d_medio = (puntos_ojos[EYES_RIGHT_EYER]  + puntos_ojos[EYES_RIGHT_EYEL] ) / 2.0;
			int x = (int)(ojo_d_medio.x - ((eyeWidth / 2)));// - 0.30 * eyeWidth));
			int y = (int)(ojo_d_medio.y - ((eyeHeight / 2)));// - 0.35 * eyeHeight));
			int width = (int)((eyeWidth));// - 0.2 * eyeWidth);
			int height = (int)((eyeHeight));// - 0.2 * eyeHeight);

			//		cout << "RIGHT ROI: " << x << ", " << y << ", " << width << ", " << height << endl;
			if (x >= 0 && y >= 0 && width > 0 && height > 0){
				Mat ROI = imagen(Rect(x, y, width, height));

				//			imshow("Image_dcho", ROI);
				//			waitKey(1);

				// Gray scale
				Mat grayImage;
				cvtColor(ROI, grayImage, CV_RGB2GRAY);

				// Normalizes brightness and increases contrast
				equalizeHist(grayImage, grayImage);

				// Black and white (inverted)
				Mat blackAndWhiteImage;
				threshold(grayImage, blackAndWhiteImage, 60, 255, THRESH_BINARY_INV); // este parametro se puede aprender
				// Bigger x4
				Mat resizeImage;
				resize(blackAndWhiteImage, resizeImage, Size(), 4.0, 4.0);

				// Erode filter
				Mat erodeImage;
				erode(resizeImage, erodeImage, Mat());
				//				Image<Gray, Byte> erodeImage = resizeImage.Erode(4);

				//				imshow("ErodeImage", erodeImage);

				Moments momentos = moments(erodeImage);
				Point2d pupil (momentos.m10 / momentos.m00, momentos.m01 / momentos.m00);

				// Eye Center = Gravity center
				//				MCvPoint2D64f pupil = erodeImage.GetMoments(true).GravityCenter;
				Point2d pupilInColorSpace;
				pupilInColorSpace.x = x + (double)(pupil.x/4.0);
				pupilInColorSpace.y = y + (double)(pupil.y/4.0);

				right_pupil = pupilInColorSpace;
				//			ojo_d.pupila = right_pupil;
				//				cout << "Pupila derecha detectada: " << right_pupil << endl;
			}
		}
	}
}

cv::Mat rot2eulerYZX(const cv::Mat & rotationMatrix) {
	cv::Mat euler(3,1,CV_64F);

	double m00 = rotationMatrix.at<double>(0,0);
	//	double m01 = rotationMatrix.at<double>(0,1);
	//	double m02 = rotationMatrix.at<double>(0,2);
	double m10 = rotationMatrix.at<double>(1,0);
	double m11 = rotationMatrix.at<double>(1,1);
	double m12 = rotationMatrix.at<double>(1,2);
	double m20 = rotationMatrix.at<double>(2,0);
	//	double m21 = rotationMatrix.at<double>(2,1);
	//	double m22 = rotationMatrix.at<double>(2,2);

	double x, y, z;
	z = asin(m10);
	x = atan2(-m12/cos(z),m11/cos(z));
	y = atan2(-m20/cos(z),m00/cos(z));

	euler.at<double>(0) = x;
	euler.at<double>(1) = y;
	euler.at<double>(2) = z;

	return euler;
}

