
#include "gaze_tracking.h"

#define VIEW 0

Mat pointsImage (1080, 1920, CV_8UC3, Scalar(0,0,0));
Mat pointsImage_wr (1080, 1920, CV_8UC3, Scalar(0,0,0));
Point3d pupila_izq, pupila_dcha, ojo_izq_izq, ojo_izq_dch, ojo_dch_izq, ojo_dch_dch;
Mat rvec, tvec;


#define RIGHT_PUPIL 12
#define LEFT_PUPIL 13

void callback(const geometry_msgs::PoseStampedConstPtr& pose,
		const tfm_msgs::Vector_PointsConstPtr& points1) {

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

	pupila_izq = puntos_ojos_3d[LEFT_PUPIL];
	pupila_dcha = puntos_ojos_3d[RIGHT_PUPIL];
	ojo_izq_izq = puntos_ojos_3d[LEFT_EYEL];
	ojo_izq_dch = puntos_ojos_3d[LEFT_EYER];
	ojo_dch_izq = puntos_ojos_3d[RIGHT_EYEL];
	ojo_dch_dch = puntos_ojos_3d[RIGHT_EYER];
}


int main (int argc, char** argv) {

	//	Point2d p1 (10, -235), p2 (-218.5, -235), p3 (10, -105), p4 (-218.5, -105);
	//	vector<Point2d> v_p; v_p.push_back(p1); v_p.push_back(p2); v_p.push_back(p3); v_p.push_back(p4);

	cv::Mat t_cam_screen = (Mat_<double>(4, 4) <<
			-0.9994,   -0.0207,    -0.0259, 140.6000,
			-0.0155,    0.9802,    -0.1975, -308.4000,
			0.0296,   -0.1970,    -0.98,    6.8000,
			0,         0,         0,         1.0000);

	vector<Point2d> v_point_screen;
	vector<Mat> v_point_cam;
	srand (time(NULL));
	for (int i = 0; i < 10; i++){
		double x = rand() % 1920;
		double y = rand() % 1080;
		Mat point_screen (4, 1, CV_64F, Scalar(0.0));
		point_screen.at<double>(0) = x  * 477 / 1920.0;
		point_screen.at<double>(1) = y  * 270 / 1080.0;
		point_screen.at<double>(2) = 0.0;
		point_screen.at<double>(3) = 1.0;

		v_point_screen.push_back(Point2d(x, y));
		Mat point_cam = t_cam_screen * point_screen;

		v_point_cam.push_back(point_cam);
	}

	vector<Point2d> v_real_point_screen;
	Mat v_real_point_cam;
	Mat v_rvec;
	Mat v_tvec;
	vector<Point3d> v_pupil_i;
	vector<Point3d> v_pupil_d;
	vector<Point3d> v_leye_l;
	vector<Point3d> v_leye_r;
	vector<Point3d> v_reye_l;
	vector<Point3d> v_reye_r;

	try{
		//	 Initializa ROS
		ros::init (argc, argv, "personal_calibration");
		ros::NodeHandle nh;

		message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose(nh, "/face/pose", 1);
		message_filters::Subscriber<tfm_msgs::Vector_Points> sub_puntos3(nh, "/eyes/points3d", 1);

		typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, tfm_msgs::Vector_Points> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_pose, sub_puntos3);
		sync.registerCallback(boost::bind(&callback, _1, _2));


		std::chrono::time_point<std::chrono::high_resolution_clock> inicio, ahora, inicio2, ahora2;
		double fps = 0;
		size_t frameCount = 0;
		std::ostringstream oss;
		namedWindow("LOOK HERE", CV_WINDOW_NORMAL);

		inicio = std::chrono::high_resolution_clock::now();
		inicio2 = std::chrono::high_resolution_clock::now();
		ros::Rate r(20); // 10 hz
		int punto_pintar = 0;
		while (nh.ok()) {
			//		 Spin
			ros::spinOnce();


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

			ahora2 = std::chrono::high_resolution_clock::now();
			double elapsed2 = std::chrono::duration_cast<std::chrono::milliseconds>(ahora2 - inicio2).count() / 1000.0;

			if (elapsed2 >= 20.0){
				punto_pintar ++;
				inicio2 = ahora2;
				pointsImage = Mat (1080, 1920, CV_8UC3, Scalar(0,0,0));
				if (punto_pintar > v_point_screen.size() - 1) break;
			}

			circle( pointsImage, v_point_screen[punto_pintar], 5, Scalar(0, 255, 255), -1);
			circle( pointsImage_wr, v_point_screen[punto_pintar], 5, Scalar(0, 255, 255), -1);
			imshow("LOOK HERE", pointsImage);
			waitKey(1);

			v_real_point_screen.push_back(v_point_screen[punto_pintar]);
			v_real_point_cam.push_back(v_point_cam[punto_pintar]);
			v_rvec.push_back(rvec.t());
			v_tvec.push_back(tvec.t());
			v_pupil_i.push_back(pupila_izq);
			v_pupil_d.push_back(pupila_dcha);
			v_leye_l.push_back(ojo_izq_izq);
			v_leye_r.push_back(ojo_izq_dch);
			v_reye_l.push_back(ojo_dch_izq);
			v_reye_r.push_back(ojo_dch_dch);

			//			cout << ros::Time::now() << ", " << pupila_izq << ". " << pupila_dcha
			//					<< ", " << rvec << ", " << tvec << endl;

			r.sleep();
		}


		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);

		imwrite("/home/ivvi2018/Escritorio/puntos.png", pointsImage_wr, compression_params);


		ofstream myfile;
		myfile.open ("/home/ivvi2018/Escritorio/calibration_data.txt");
		//		myfile << "[";

		myfile << "v_real_point_screen = [" ;
		for (int i = 0; i < v_real_point_screen.size(); i++){
			myfile << v_real_point_screen[i].x << " " << v_real_point_screen[i].y << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_pupil_i = [" ;
		for (int i = 0; i < v_pupil_i.size(); i++){
			myfile << v_pupil_i[i].x << " " << v_pupil_i[i].y << " " << v_pupil_i[i].z << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_pupil_d = [" ;
		for (int i = 0; i < v_pupil_d.size(); i++){
			myfile << v_pupil_d[i].x << " " << v_pupil_d[i].y << " " << v_pupil_d[i].z << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_leye_l = [" ;
		for (int i = 0; i < v_leye_l.size(); i++){
			myfile << v_leye_l[i].x << " " << v_leye_l[i].y << " " << v_leye_l[i].z << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_leye_r = [" ;
		for (int i = 0; i < v_leye_r.size(); i++){
			myfile << v_leye_r[i].x << " " << v_leye_r[i].y << " " << v_leye_r[i].z << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_reye_l = [" ;
		for (int i = 0; i < v_reye_l.size(); i++){
			myfile << v_reye_l[i].x << " " << v_reye_l[i].y << " " << v_reye_l[i].z << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_reye_r = [" ;
		for (int i = 0; i < v_reye_r.size(); i++){
			myfile << v_reye_r[i].x << " " << v_reye_r[i].y << " " << v_reye_r[i].z << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_rvec = [" ;
		for (int i = 0; i < v_rvec.rows; i++){
			myfile << v_rvec.at<double>(i,0) << " " << v_rvec.at<double>(i,1) << " " << v_rvec.at<double>(i,2) << ";" << endl;
		}
		myfile << "];" << endl;

		myfile << "v_tvec = [" ;
		for (int i = 0; i < v_tvec.rows; i++){
			myfile << v_tvec.at<double>(i,0) << " " << v_tvec.at<double>(i,1) << " " << v_tvec.at<double>(i,2) << ";" << endl;
		}
		myfile << "];" << endl;

		myfile.close();


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
