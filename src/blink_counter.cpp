
#include "blink_counter.h"

#define VIEW 0
Mat depthData, imageRGB;
vector<Point3f> puntos_ojos_3d;
vector<Point2i> puntos_ojos_2d;
ojo ojo_derecho;
ojo ojo_izquierdo;

void get_points3 (const tfm_msgs::Vector_Points& input) {
	puntos_ojos_3d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < input.num_puntos.data; i++){
		Point3f punto_ojos;
		punto_ojos.x = input.puntos[i].x;
		punto_ojos.y = input.puntos[i].y;
		punto_ojos.z = input.puntos[i].z;
		puntos_ojos_3d.push_back(punto_ojos);
	}

}

void get_points2 (const tfm_msgs::Vector_Points& input) {
	puntos_ojos_2d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < input.num_puntos.data; i++){
		Point2i punto_ojos;
		punto_ojos.x = input.puntos[i].x;
		punto_ojos.y = input.puntos[i].y;
		puntos_ojos_2d.push_back(punto_ojos);
	}

	ojo_izquierdo.corner_dcho = puntos_ojos_2d[LEFT_EYER];
	ojo_izquierdo.corner_izdo = puntos_ojos_2d[LEFT_EYEL];
	ojo_izquierdo.liminf_d = puntos_ojos_2d[LEFT_EYE_TOP_R];
	ojo_izquierdo.liminf_i = puntos_ojos_2d[LEFT_EYE_TOP_L];
	ojo_izquierdo.limsup_d = puntos_ojos_2d[LEFT_EYE_BOTTOM_R];
	ojo_izquierdo.limsup_i = puntos_ojos_2d[LEFT_EYE_BOTTOM_L];

	ojo_derecho.corner_dcho = puntos_ojos_2d[RIGHT_EYER];
	ojo_derecho.corner_izdo = puntos_ojos_2d[RIGHT_EYEL];
	ojo_derecho.liminf_d = puntos_ojos_2d[RIGHT_EYE_TOP_R];
	ojo_derecho.liminf_i = puntos_ojos_2d[RIGHT_EYE_TOP_L];
	ojo_derecho.limsup_d = puntos_ojos_2d[RIGHT_EYE_BOTTOM_R];
	ojo_derecho.limsup_i = puntos_ojos_2d[RIGHT_EYE_BOTTOM_L];
}

void callback(const tfm_msgs::Vector_PointsConstPtr& points1, const tfm_msgs::Vector_PointsConstPtr& points2) {

	puntos_ojos_3d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < points1->num_puntos.data; i++){
		Point3f punto_ojos;
		punto_ojos.x = points1->puntos[i].x;
		punto_ojos.y = points1->puntos[i].y;
		punto_ojos.z = points1->puntos[i].z;
		puntos_ojos_3d.push_back(punto_ojos);
	}


	puntos_ojos_2d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < points2->num_puntos.data; i++){
		Point2f punto_ojos;
		punto_ojos.x = points2->puntos[i].x;
		punto_ojos.y = points2->puntos[i].y;
		puntos_ojos_2d.push_back(punto_ojos);
	}
}

int main (int argc, char** argv) {
	vector<Point3f> puntos_ojos_3D;
	vector<Point2i> puntos_ojos_2D;

	// initialize the frame counters and the total number of blinks
	int blink_counter = 0;
	int total_blinks = 0;

	try{
		//	 Initializa ROS
		ros::init (argc, argv, "blink_counter");
		ros::NodeHandle nh;

		//	 Crear subscriptores para la nube de puntos y la imagen de color
//		ros::Subscriber sub_puntos3 = nh.subscribe ("/eyes/points3d", 1, get_points3);
//		ros::Subscriber sub_puntos2 = nh.subscribe ("/eyes/points2d", 1, get_points2);

		message_filters::Subscriber<tfm_msgs::Vector_Points> sub_puntos3(nh, "/eyes/points3d", 1);
		message_filters::Subscriber<tfm_msgs::Vector_Points> sub_puntos2(nh, "/eyes/points2d", 1);

		typedef message_filters::sync_policies::ApproximateTime<tfm_msgs::Vector_Points, tfm_msgs::Vector_Points> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_puntos3, sub_puntos2);
		sync.registerCallback(boost::bind(&callback, _1, _2));

		//		ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>("/face/pose", 1);

		std::chrono::time_point<std::chrono::high_resolution_clock> inicio, ahora;
		double fps = 0;
		size_t frameCount = 0;
		std::ostringstream oss;

		//			Esperar hasta obtener los subscriptores
//		while (sub_puntos3.getNumPublishers() == 0 && sub_puntos2.getNumPublishers() == 0) {
//			ROS_ERROR("Waiting for publishers");
//			sleep(1);
//		}
//		ROS_INFO("Got publisher");

		inicio = std::chrono::high_resolution_clock::now();
		ros::Rate r(20); // 10 hz
		while (nh.ok()) {
			//		 Spin
			ros::spinOnce();

			Mat detected_eyes (540, 960, CV_8UC3, Scalar(255, 255, 0));

			if (puntos_ojos_3d.size() > 0){
				if (VIEW) DibujarOjos(detected_eyes, puntos_ojos_2d, Scalar(0, 0, 255));

				float right_ear = eye_aspect_ratio(ojo_derecho);
				float left_ear = eye_aspect_ratio(ojo_izquierdo);
				float ear = (right_ear + left_ear) / 2.0;
				//				cout << "EAR: " << ear << endl;

				if (ear < EYE_AR_THRESH){
					blink_counter ++;

					// otherwise, the eye aspect ratio is not below the blink
					// threshold
				} else {
					// if the eyes were closed for a sufficient number of
					// then increment the total number of blinks
					if (blink_counter >= EYE_AR_CONSEC_FRAMES){
						total_blinks ++;
						cout << "Blinks: " << total_blinks << endl;
						// reset the eye frame counter
						blink_counter = 0;
					}
				}
				// draw the total number of blinks on the frame along with
				// the computed eye aspect ratio for the frame
				if (VIEW) putText(detected_eyes, "Blinks: " + to_string(total_blinks), Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255,0), 1, 8, false);
				if (VIEW) putText(detected_eyes, "EAR: " + to_string(ear), Point(300, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255,0), 1, 8, false);


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

				if (VIEW) cv::putText(detected_eyes, oss.str(), Point(700,30), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 255), 1, 8, false);

				if (VIEW) namedWindow("Ojos", CV_WINDOW_AUTOSIZE);
				if (VIEW) imshow("Ojos", detected_eyes);
				waitKey(1);
			}
			r.sleep();
		}

	} catch(exception& e) {
		cout << e.what() << endl;
	}

	return(0);
}


void DibujarOjos(cv::Mat &image, vector<Point2i> ojos, Scalar color){
	//	cout << "DibujarRostro " << cara.size() << endl;
	// Around Chin. Ear to Ear
	//	for (unsigned long i = 1; i <= 16; ++i)
	//		cv::line(image, Point(shape.part(i).x(), shape.part(i).y()), Point(shape.part(i-1).x(), shape.part(i-1).y()), cv::Scalar(255,255,0,0), 1, 8, 0);

	if (ojos.size() > 11){
		// Left eye
		for (unsigned long i = 1; i <= 5; ++i)
			cv::line(image, Point(ojos[i].x, ojos[i].y), Point(ojos[i-1].x, ojos[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(ojos[0].x, ojos[0].y), Point(ojos[5].x, ojos[5].y), color, 1, 8, 0);

		// Right eye
		for (unsigned long i = 7; i <= 11; ++i)
			cv::line(image, Point(ojos[i].x, ojos[i].y), Point(ojos[i-1].x, ojos[i-1].y), color, 1, 8, 0);
		cv::line(image, Point(ojos[6].x, ojos[6].y), Point(ojos[11].x, ojos[11].y), color, 1, 8, 0);
	}
}

void DibujarEjes(cv::Mat &cam, cv::Mat & tran) {
	cv::Point3f Pto3D_camera;
	std::vector<cv::Point3f> PointsEjes;
	Mat Pto3D_H;

	//Se definen los puntos de los ejes tridimensionalmente y se transforman para que el origen de coordenadas
	//aparezca entre los ojos de la cara detectada
	Pto3D_H = (Mat_<double>(4,1) << 0, 0, 0, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0);// / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1);// / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2);// / 1000.0;
	PointsEjes.push_back(Pto3D_camera);

	Pto3D_H = (Mat_<double>(4,1) << 100, 0, 0, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0);// / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1);// / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2);// / 1000.0;
	PointsEjes.push_back(Pto3D_camera);

	Pto3D_H = (Mat_<double>(4,1) << 0, 100, 0, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0);// / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1);// / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2);// / 1000.0;
	PointsEjes.push_back(Pto3D_camera);

	Pto3D_H = (Mat_<double>(4,1) << 0, 0, 100, 1);
	Pto3D_H = tran*Pto3D_H;
	Pto3D_camera.x = Pto3D_H.at<double>(0);// / 1000.0;
	Pto3D_camera.y = Pto3D_H.at<double>(1);// / 1000.0;
	Pto3D_camera.z = Pto3D_H.at<double>(2);// / 1000.0;
	PointsEjes.push_back(Pto3D_camera);


	//Se proyectan los puntos tridimensionales en la imagen
	std::vector<cv::Point2f>  Pto2DEjes;
	projectPoints(PointsEjes, /*cameraRot*/ cameraT_z, cameraT_z, cameraMatrix_RGB, distCoeffs_RGB, Pto2DEjes, cv::noArray(), 0);

	//	for (int i = 0; i < Pto2DEjes.size(); i++){
	//		Pto2DEjes[i] = Point(Pto2DEjes[i].x /*- top_left_corner.x*/, Pto2DEjes[i].y /*- top_left_corner.y*/);
	//		if (Pto2DEjes[i].x < 0 || Pto2DEjes[i].x > cam.cols || Pto2DEjes[i].y < 0 || Pto2DEjes[i].y > cam.rows)
	//			cout << "Eje fuera de imagen" << endl;
	//	}

	//Se dibujan los ejes en la imagen
	cv::line(cam, Pto2DEjes[0],Pto2DEjes[1], cv::Scalar(255,0,0,0), 1, 8, 0);
	putText(cam, "X", Pto2DEjes[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0), 1, 8, false);

	cv::line(cam, Pto2DEjes[0],Pto2DEjes[2], cv::Scalar(0,255,0,0), 1, 8, 0);
	putText(cam, "Y", Pto2DEjes[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0), 1, 8, false);

	cv::line(cam, Pto2DEjes[0],Pto2DEjes[3], cv::Scalar(0,0,255,0), 1, 8, 0);
	putText(cam, "Z", Pto2DEjes[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0), 1, 8, false);
}

float eye_aspect_ratio(ojo & p_ojo){
	//	 compute the euclidean distances between the two sets of
	//	 vertical eye landmarks (x, y)-coordinates
	float A = distanceP2P(p_ojo.limsup_i, p_ojo.liminf_i);
	float B = distanceP2P(p_ojo.limsup_d, p_ojo.liminf_d);

	//	compute the euclidean distance between the horizontal
	//	 eye landmark (x, y)-coordinates
	float C = distanceP2P(p_ojo.corner_izdo, p_ojo.corner_dcho);

	//	compute the eye aspect ratio
	float ear = (A + B) / (2.0 * C);

	//	return the eye aspect ratio
	//	cout << "ear: " << ear << endl;
	return ear;
}

