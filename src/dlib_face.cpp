
#include "dlib_face.h"

#define VIEW 0
pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
Mat depthData, imageRGB, imageRGB_big;

void callback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud_aux;

	pcl::fromROSMsg(*input_cloud, cloud_aux);
	_cloud = cloud_aux.makeShared();

	depthData = Mat(_cloud->height, _cloud->width, CV_32FC3, Scalar(0.0, 0.0, 0.0));

	for (int i = 0, v = 0; v < _cloud->height; v++){
		for (int u = 0; u < _cloud->width; u++, i++){
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
}


/**
// * Obtiene la nube de puntos de la cámara
// */
//void get_cloud (const sensor_msgs::PointCloud2 input) {
//	pcl::PointCloud<pcl::PointXYZRGB> cloud_aux;
//
//	pcl::fromROSMsg(input, cloud_aux);
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
//
//void getRGBimage (const sensor_msgs::Image& input){
//	cv_bridge::CvImagePtr img;
//	img = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8); // container of type sensor_msgs/Image
//	imageRGB = img->image;
//	//	imshow("Imagen", imageRGB);
//	//	waitKey(1);
//}
//
//void getRGBimage_big (const sensor_msgs::Image& input){
//	cv_bridge::CvImagePtr img;
//	img = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8); // container of type sensor_msgs/Image
//	imageRGB_big = img->image;
//	//	imshow("Imagen", imageRGB);
//	//	waitKey(1);
//}


int main (int argc, char** argv) {

	struct Face driver;
	vector<Point3f> puntos_cara_3D;
	vector<Point2i> puntos_cara_2D;
	vector<Point3f> puntos_ojos_3D;
	vector<Point2i> puntos_ojos_2D;

	cv::Rect roi_sig (0, 0, 960, 540);
	cv::Rect roi_anterior (0, 0, 960, 540);

	//	cv::Rect roi_sig (0, 0, 1920, 1080);
	//	cv::Rect roi_anterior (0, 0, 1920, 1080);

	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	dlib::shape_predictor pose_model;
	dlib::deserialize("/home/ivvi2018/catkin_ws/src/face_detection/shape_predictor_68_face_landmarks.dat") >> pose_model;
	//	dlib::image_window win;

	try{
		//	 Initializa ROS
		ros::init (argc, argv, "dlib_face");
		ros::NodeHandle nh;
		std_msgs::String msg;

		//	 Crear subscriptores para la nube de puntos y la imagen de color
		//		ros::Subscriber sub_color_big = nh.subscribe ("/kinect2/hd/image_color_rect", 1, getRGBimage_big);
		//		ros::Subscriber sub_color = nh.subscribe ("/kinect2/qhd/image_color_rect", 1, getRGBimage);
		//		ros::Subscriber sub_points  = nh.subscribe ("/kinect2/qhd/points", 1, get_cloud);

		message_filters::Subscriber<sensor_msgs::Image> sub_color(nh, "/kinect2/qhd/image_color_rect", 1);
		message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points(nh, "/kinect2/qhd/points", 1);

		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_color, sub_points);
		sync.registerCallback(boost::bind(&callback, _1, _2));

		ros::Publisher pub_img = nh.advertise<sensor_msgs::Image>("/face/image_ROI", 1);
		//		ros::Publisher pub_depth_img = nh.advertise<sensor_msgs::Image>("/face/image_depth_ROI", 1);
		ros::Publisher pub_points3d = nh.advertise<tfm_msgs::Vector_Points>("/face/points3d", 1);
		ros::Publisher pub_points2d = nh.advertise<tfm_msgs::Vector_Points>("/face/points2d", 1);
		ros::Publisher pub_points3d_eyes = nh.advertise<tfm_msgs::Vector_Points>("/eyes/points3d", 1);
		ros::Publisher pub_points2d_eyes = nh.advertise<tfm_msgs::Vector_Points>("/eyes/points2d", 1);

		std::chrono::time_point<std::chrono::high_resolution_clock> inicio, ahora;
		double fps = 0;
		size_t frameCount = 0;
		std::ostringstream oss;

		//			Esperar hasta obtener los subscriptores
		//		while ((sub_color.getNumPublishers() == 0) && (sub_points.getNumPublishers() == 0)) {
		//			ROS_ERROR("Waiting for publishers");
		//			sleep(1);
		//		}
		//		ROS_INFO("Got publisher");

		inicio = std::chrono::high_resolution_clock::now();
		ros::Rate r(20); // 10 hz
		while (nh.ok()) {
			//		 Spin
			ros::spinOnce();

			if (!imageRGB.empty() && !depthData.empty()){
				Mat depthData_small(1, 1, CV_32FC3), imageRGB_small(1, 1, CV_8UC3), faceRGB_small(1, 1, CV_8UC3);

				//				Rect roi_big (roi_anterior.x * 2.0, roi_anterior.y * 2.0, roi_anterior.width * 2.0, roi_anterior.height * 2.0);
				imageRGB_small = imageRGB(roi_anterior);
				depthData_small = depthData(roi_anterior);
				//				faceRGB_small = imageRGB_big(roi_big);

				//				if (VIEW) {
				//					imshow("face", faceRGB_small);
				//					waitKey(1);
				//				}

				sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageRGB).toImageMsg();
				pub_img.publish(msg_img);
				//				sensor_msgs::ImagePtr msg_img_depth = cv_bridge::CvImage(std_msgs::Header(), "TYPE_32FC3", depthData).toImageMsg();
				//				pub_depth_img.publish(msg_img_depth);

				Point v1 (roi_anterior.x, roi_anterior.y);
				Point v2 (roi_anterior.x + roi_anterior.width, roi_anterior.y + roi_anterior.height);
				if (VIEW) {
					rectangle(imageRGB, v1, v2, Scalar(0, 0, 255), 2);
					namedWindow("ROI", CV_WINDOW_AUTOSIZE);
					imshow("ROI", imageRGB);
					waitKey(1);
				}

				dlib::cv_image<dlib::bgr_pixel> cimg(imageRGB_small);
				std::vector<dlib::rectangle> faces = detector(cimg);
				std::vector<dlib::full_object_detection> shapes;
				for (unsigned long i = 0; i < faces.size(); ++i){
					shapes.push_back(pose_model(cimg, faces[i]));

					roi_sig = Rect(roi_anterior.x + faces[i].left() - faces[i].width() / 2 , roi_anterior.y + faces[i].top() - faces[i].height() / 2.0, 2*faces[i].width(), 2*faces[i].height());
					//					cout << "[" << roi_sig.x << ", " << roi_sig.y << "] [" << roi_sig.width << ", " << roi_sig.height << "]" << endl;

					if(roi_sig.x < 0){
						roi_sig.width += roi_sig.x;
						roi_sig.x = 0;
					}
					if(roi_sig.y < 0){
						roi_sig.height += roi_sig.y;
						roi_sig.y = 0;
					}
				}

				tfm_msgs::Vector_Points points3_msg;
				tfm_msgs::Vector_Points points2_msg;
				tfm_msgs::Vector_Points points3_eyes_msg;
				tfm_msgs::Vector_Points points2_eyes_msg;

				if(faces.empty()){
					driver.presencia = 0;
					continue;
				} else {
					driver.presencia = 1;

					Point2d right_pupil, left_pupil;

					puntos_cara_3D.clear(); puntos_cara_2D.clear();
					puntos_ojos_3D.clear(); puntos_ojos_2D.clear();
					ObtienePuntos(puntos_cara_3D, puntos_cara_2D, puntos_ojos_3D, puntos_ojos_2D, shapes, driver, depthData_small);
					PupilDetection(puntos_ojos_2D, right_pupil, left_pupil, imageRGB_small);

					puntos_cara_2D.push_back(right_pupil);
					puntos_ojos_2D.push_back(right_pupil);
					Point3f right_pupil_w = depthData_small.at<Vec3f>(right_pupil.y,right_pupil.x);
					puntos_cara_3D.push_back(right_pupil_w);
					puntos_ojos_3D.push_back(right_pupil_w);

					puntos_cara_2D.push_back(left_pupil);
					puntos_ojos_2D.push_back(left_pupil);
					Point3f left_pupil_w = depthData_small.at<Vec3f>(left_pupil.y,left_pupil.x);
					puntos_cara_3D.push_back(left_pupil_w);
					puntos_ojos_3D.push_back(left_pupil_w);

					points3_msg.puntos.clear();
					points2_msg.puntos.clear();
					points3_eyes_msg.puntos.clear();
					points2_eyes_msg.puntos.clear();

					for (int i = 0; i < (int)puntos_cara_2D.size(); i++){
						geometry_msgs::Point punto_cara;
						punto_cara.x = puntos_cara_3D[i].x;
						punto_cara.y = puntos_cara_3D[i].y;
						punto_cara.z = puntos_cara_3D[i].z;
						points3_msg.puntos.push_back(punto_cara);
						punto_cara.x = puntos_cara_2D[i].x + roi_anterior.x;
						punto_cara.y = puntos_cara_2D[i].y + roi_anterior.y;
						punto_cara.z = 0.0;
						points2_msg.puntos.push_back(punto_cara);
					}

					for (int i = 0; i < (int)puntos_ojos_2D.size(); i++){
						geometry_msgs::Point punto_ojos;
						punto_ojos.x = puntos_ojos_3D[i].x;
						punto_ojos.y = puntos_ojos_3D[i].y;
						punto_ojos.z = puntos_ojos_3D[i].z;
						points3_eyes_msg.puntos.push_back(punto_ojos);
						punto_ojos.x = puntos_ojos_2D[i].x + roi_anterior.x;
						punto_ojos.y = puntos_ojos_2D[i].y + roi_anterior.y;
						punto_ojos.z = 0.0;
						points2_eyes_msg.puntos.push_back(punto_ojos);
					}

					std_msgs::Int32 n_puntos;
					n_puntos.data = puntos_cara_2D.size();
					points3_msg.num_puntos = n_puntos;
					points2_msg.num_puntos = n_puntos;

					std_msgs::Int32 n_puntos_ojos;
					n_puntos_ojos.data = puntos_ojos_2D.size();
					points3_eyes_msg.num_puntos = n_puntos_ojos;
					points2_eyes_msg.num_puntos = n_puntos_ojos;
					//					cout << "filled!" << endl;
				}

				points3_msg.header.stamp = ros::Time::now();
				points2_msg.header.stamp = ros::Time::now();
				points3_eyes_msg.header.stamp = ros::Time::now();
				points2_eyes_msg.header.stamp = ros::Time::now();

				pub_points3d.publish(points3_msg);
				pub_points2d.publish(points2_msg);
				pub_points3d_eyes.publish(points3_eyes_msg);
				pub_points2d_eyes.publish(points2_eyes_msg);
				//				cout << "published!" << endl;
				roi_anterior = roi_sig;

				++frameCount;
				ahora = std::chrono::high_resolution_clock::now();
				double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(ahora - inicio).count() / 1000.0;

				if(elapsed >= 1.0) {
					fps = frameCount / elapsed;
					oss.str("");
					oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
					inicio = ahora;
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

	imageRGB.release();
	depthData.release();

	return(0);
}

//Almacena en Points3D y Points2D los puntos detectados de la cara
//En Points2D se almacena la información en píxeles y en Points3D la información en milimetros
void ObtienePuntos(std::vector<cv::Point3f> &Points3D, std::vector<cv::Point2i> &Points2D, vector<cv::Point3f> &PointsEyes3D, vector<cv::Point2i> &PointsEyes2D, std::vector<dlib::full_object_detection> & shapes, Face & driver, Mat & depth){

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

	for (int i = 0; i < puntos_cara.size(); i++){
		Pto_Im.x = shapes[0].part(puntos_cara[i]).x();
		Pto_Im.y = shapes[0].part(puntos_cara[i]).y();
		Pto3D = depth.at<Vec3f>(Pto_Im.y,Pto_Im.x);
		Points2D.push_back(Pto_Im);
		Points3D.push_back(Pto3D);
	}
	//	cout << Points2D.size() << ", " << Points3D.size() << ", " << PointsEyes2D.size() << ", " << PointsEyes3D.size() << endl;

	for (int i = RIGHT_EYER; i <= LEFT_EYE_BOTTOM_R; i++){
		PointsEyes2D.push_back(Points2D[i]);
		PointsEyes3D.push_back(Points3D[i]);
	}
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
