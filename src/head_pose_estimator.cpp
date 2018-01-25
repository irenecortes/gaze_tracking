
#include "head_pose_estimator.h"

#define VIEW 1

Mat depthData, imageRGB, transf (3, 4, CV_64FC1);
int tamano_filtro = 11;
vector<Point3f> puntos_cara_3d;
vector<Point2f> puntos_cara_2d;

void get_points3 (const tfm_msgs::Vector_Points& input) {
	puntos_cara_3d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < input.num_puntos.data; i++){
		Point3f punto_cara;
		punto_cara.x = input.puntos[i].x;
		punto_cara.y = input.puntos[i].y;
		punto_cara.z = input.puntos[i].z;
		puntos_cara_3d.push_back(punto_cara);
	}
	//	cout << "fin get_points" << endl;
}

void get_points2 (const tfm_msgs::Vector_Points& input) {
	puntos_cara_2d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < input.num_puntos.data; i++){
		Point2f punto_cara;
		punto_cara.x = input.puntos[i].x;
		punto_cara.y = input.puntos[i].y;
		puntos_cara_2d.push_back(punto_cara);
	}
	//	cout << "fin get_points" << endl;
}

void callback(const tfm_msgs::Vector_PointsConstPtr& points1, const tfm_msgs::Vector_PointsConstPtr& points2) {

	puntos_cara_2d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	for (int i = 0; i < points2->num_puntos.data; i++){
		Point2f punto_cara;
		punto_cara.x = points2->puntos[i].x;
		punto_cara.y = points2->puntos[i].y;
		puntos_cara_2d.push_back(punto_cara);
	}

	// puntos_cara_3d.clear();
	//	cout << "get_points " << input.num_puntos.data << endl;
	if (puntos_cara_3d.size() == 0){
		for (int i = 0; i < points1->num_puntos.data; i++){
			Point3f punto_cara;
			punto_cara.x = points1->puntos[i].x;
			punto_cara.y = points1->puntos[i].y;
			punto_cara.z = points1->puntos[i].z;
			puntos_cara_3d.push_back(punto_cara);
		}
	} else {
		for (int i = 0; i < points1->num_puntos.data; i++){
			Point3f punto_cara;
			punto_cara.x = points1->puntos[i].x;
			punto_cara.y = points1->puntos[i].y;
			punto_cara.z = points1->puntos[i].z;
			if (punto_cara != Point3f(0.0,0.0,0.0)){
				puntos_cara_3d[i] = punto_cara;
			}
		}
	}

}

int main (int argc, char** argv) {

	vector<Point3f> puntos_cara_3D;
	vector<Point2i> puntos_cara_2D;

	vector<Mat> rvecs, tvecs;

	//	namedWindow("Cara detectada", CV_WINDOW_NORMAL);
	ofstream myfile;
	myfile.open ("/home/irenerrrd/Escritorio/pose_estimation.txt");

	try{
		//	 Initializa ROS
		ros::init (argc, argv, "head_pose_estimator");
		ros::NodeHandle nh;

		//	 Crear subscriptores para la nube de puntos y la imagen de color
		//		ros::Subscriber sub_puntos3 = nh.subscribe ("/face/points3d", 1, get_points3);
		//		ros::Subscriber sub_puntos2 = nh.subscribe ("/face/points2d", 1, get_points2);

		message_filters::Subscriber<tfm_msgs::Vector_Points> sub_puntos3(nh, "/face/points3d", 1);
		message_filters::Subscriber<tfm_msgs::Vector_Points> sub_puntos2(nh, "/face/points2d", 1);

		typedef message_filters::sync_policies::ApproximateTime<tfm_msgs::Vector_Points, tfm_msgs::Vector_Points> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_puntos3, sub_puntos2);
		sync.registerCallback(boost::bind(&callback, _1, _2));

		ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/face/pose", 1);

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

		std::vector<cv::Point3f> Points_ideal, Points_ideal_dib;
		std::vector<cv::Point3f> Points_real;

		Inicializar3D_Ideal(Points_ideal);

		while (nh.ok()) {
			//		 Spin
			ros::spinOnce();

			Mat detected_face_im (540, 960, CV_8UC3, Scalar(0, 0, 0));

			if (puntos_cara_3d.size() > 0){
				if (VIEW) DibujarRostro(detected_face_im, puntos_cara_2d, Scalar(0, 0, 255));

				//				std::vector<cv::Point2i> Points2d_real;
				Inicializar3D_Real(Points_real);
				//				Inicializar2D_Real(Points2d_real);
				//				cout << "Points3D: " << endl;
				//				for (int i = 0; i < Points_real.size(); i++){
				//					cout << Points_real[i] << endl;
				//				}
				//				cout << "PointsIdeal:" << endl;
				//				for (int i = 0; i < Points_ideal.size(); i++){
				//					cout << Points_ideal[i] << endl;
				//				}
				//				cout << "Points_real: " << Points_real.size() << endl;
				//				cout << "Points_ideal: " << Points_ideal.size() << endl;
				//				cv::Mat rvec_2d,tvec_2d;
				//				int iterationsCount = 1000;
				//				float reprojectionError = 1;
				//				double confidence = 0.9;
				//				//int flags=cv::SOLVEPNP_ITERATIVE;
				//				//int flags=cv::SOLVEPNP_P3P;
				//				//int flags=cv::SOLVEPNP_EPNP; // MUY MALO
				//				int flags=cv::SOLVEPNP_DLS;
				//				//int flags=cv::SOLVEPNP_UPNP; //MUY MALO
				//				//solvePnPRansac(Points3D, Points2D, cameraMatrix, distCoeffs, rvec, tvec, false,iterationsCount, reprojectionError,confidence, cv::noArray(), flags);
				//
				//				cv::Mat tvec_1(3, 1, CV_64FC1, Scalar(0.0)), rvec_1(3, 1, CV_64FC1, Scalar(0.0));
				//
				//				solvePnP(Points_ideal, Points2d_real, cameraMatrix_RGB, distCoeffs_null, rvec_1, tvec_1, false, flags );
				//				cv::Mat  rotation_1;
				//				Rodrigues(rvec_1, rotation_1);


				cv::Mat inliers;
				estimateAffine3D(Points_ideal, Points_real, transf, inliers, 3, 0.98);

				cv::Mat rotation (3, 3, CV_64FC1, Scalar(0.0)), tvec(3, 1, CV_64FC1, Scalar(0.0)), rvec(3, 1, CV_64FC1, Scalar(0.0));
				ObtieneTransfVec(transf, rotation, rvec, tvec);
				Mat rvec_filtered(3, 1, CV_64FC1, Scalar(0.0)), tvec_filtered(3, 1, CV_64FC1, Scalar(0.0));
				filter(rvec, tvec, rvecs, tvecs, rvec_filtered, tvec_filtered);
				ObtieneTransfMat(transf, rotation, rvec_filtered, tvec_filtered);

				//				cout << "RVec: " << rvec << endl << "TVec: " << tvec << endl;
				myfile << rvec_filtered.at<double>(0) << ", " << rvec_filtered.at<double>(1) << ", " << rvec_filtered.at<double>(2) << ", ";
				myfile << tvec_filtered.at<double>(0) << ", " << tvec_filtered.at<double>(1) << ", " << tvec_filtered.at<double>(2) << endl;

				if (VIEW) proyectarRostro(detected_face_im, Points_real, Points_ideal, transf);
				if (VIEW) DibujarEjes(detected_face_im, transf);

				geometry_msgs::PoseStamped pose_msg;
				pose_msg.pose.position.x = tvec_filtered.at<double>(0);
				pose_msg.pose.position.y = tvec_filtered.at<double>(1);
				pose_msg.pose.position.z = tvec_filtered.at<double>(2);

				tf::Quaternion q = tf::createQuaternionFromRPY(rvec_filtered.at<double>(0), rvec_filtered.at<double>(1), rvec_filtered.at<double>(2));
				pose_msg.pose.orientation.x = q.getX();
				pose_msg.pose.orientation.y = q.getY();
				pose_msg.pose.orientation.z = q.getZ();
				pose_msg.pose.orientation.w = q.getW();

				pose_msg.header.stamp = ros::Time::now();

				pub_pose.publish(pose_msg);

				Point3f origen_cara (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

				// cout << puntos_cara_3d[RIGHT_PUPIL] - origen_cara << endl
				//	 <<	puntos_cara_3d[LEFT_PUPIL] - origen_cara << endl;

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

				if (VIEW) namedWindow("Cara", CV_WINDOW_AUTOSIZE);
				if (VIEW) imshow("Cara", detected_face_im);
				if (VIEW) waitKey(1);
			}

			r.sleep();
		}

	} catch(exception& e) {
		cout << e.what() << endl;
	}

	myfile.close();
	return(0);
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

	//	cout << "RT enviada: " << transf << endl;
}

//Obtiene la matriz de rotación y los vectores de rotación y traslación. Convierte la matriz de transformación en una 4x4 con la que se puede operar
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

void filter(Mat & rvec, Mat & tvec, vector<Mat> & rvecs, vector<Mat> & tvecs, Mat & rvec_f, Mat & tvec_f){
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

void DibujarRostro(cv::Mat &image, vector<Point2f> cara, Scalar color){
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
