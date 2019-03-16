/**************************************************************************//**
   @author  Markus Lamprecht
   @date    March 2019
   @link    www.simact.de/about_me
   @Copyright (c) 2019 Markus Lamprecht. BSD
 *****************************************************************************/
 
#include <csignal>
#include <iostream>
#include <map> // used for hashmap to give certainty
#include <vector> // used in hashmap
#include <numeric> // used for summing a vector

// ROS
#include "ros/ros.h"

// ROS sensor messages
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"

// ROS image geometry
#include <image_geometry/pinhole_camera_model.h>

// ROS transform
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <alfons_msgs/ArucoInfo.h>

// ROS CvBridge
#include "cv_bridge/cv_bridge.h"

// Image Transport to publish output img
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace sensor_msgs;
using namespace cv;

// Publisher
image_transport::Publisher result_img_pub_;
ros::Publisher tf_list_pub_;
ros::Publisher aruco_info_pub_;


#define SSTR(x) static_cast<std::ostringstream&>(std::ostringstream() << std::dec << x).str()
#define ROUND2(x) std::round(x * 100) / 100
#define ROUND3(x) std::round(x * 1000) / 1000

// Define global variables
bool camera_model_computed = false;
bool show_detections;
float marker_size;
image_geometry::PinholeCameraModel camera_model;
Mat distortion_coefficients;
Matx33d intrinsic_matrix;
Ptr<aruco::DetectorParameters> detector_params;
Ptr<cv::aruco::Dictionary> dictionary;
string marker_tf_prefix;
int blur_window_size = 7;
int image_fps = 30;
int image_width = 640;
int image_height = 480;
bool enable_blur = true;

// hashmap used for uncertainty:
int num_detected = 10;  // =0 -> not used
int min_prec_value = 80; // min precentage value to be a detected marker.
map<int,  std::vector<int>  > ids_hashmap;   // key: ids, value: number within last 100 imgs


void int_handler(int x) {
    // disconnect and exit gracefully
    if (show_detections) {
        cv::destroyAllWindows();
    }
    ros::shutdown();
    exit(0);
}

tf2::Vector3 cv_vector3d_to_tf_vector3(const Vec3d &vec) {
    return {vec[0], vec[1], vec[2]};
}

double getPrec(std::vector<int> ids, int i){
 	vector<int> current_vector(num_detected);
	current_vector = ids_hashmap[ids[i]];
        int num_detections = std::accumulate(current_vector.begin(), current_vector.end(), 0);
	return (double) num_detections/num_detected *100;
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector) {
    Mat rotation_matrix;
    auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
}

tf2::Transform create_transform(const Vec3d &tvec, const Vec3d &rotation_vector) {
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
    return transform;
}

void callback_camera_info(const CameraInfoConstPtr &msg) {
    if (camera_model_computed) {
        return;
    }
    camera_model.fromCameraInfo(msg);
    camera_model.distortionCoeffs().copyTo(distortion_coefficients);
    intrinsic_matrix = camera_model.intrinsicMatrix();
    camera_model_computed = true;
    ROS_INFO("camera model is computed");
}

void update_params_cb(const std_msgs::Empty &msg)
{
	// update the parameters:
	//nh.getParam("/blur_window_size", blur_window_size);
} 

void callback(const ImageConstPtr &image_msg) {
    if (!camera_model_computed) {
        ROS_INFO("camera model is not computed yet");
        return;
    }

    string frame_id = image_msg->header.frame_id;
    auto image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat display_image(image);

    // Smooth the image to improve detection results
    if (enable_blur) {
        GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0,
                     0);
    }

    // Detect the markers
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    aruco::detectMarkers(image, dictionary, corners, ids, detector_params, rejected);

    // publish aruco info:
    alfons_msgs::ArucoInfo ar_msg;
    for(int i = 0;i<ids.size();i++)
    {
        //std_msgs::Int16 id_num = ;
        vector<Point2f> one_corner = corners[i];
        ar_msg.marker_ids.push_back(ids[i]);
        ar_msg.center_x_px.push_back((one_corner[0].x+one_corner[1].x+one_corner[2].x+one_corner[3].x)/4);
        ar_msg.center_y_px.push_back((one_corner[0].y+one_corner[1].y+one_corner[2].y+one_corner[3].y)/4);
    }
    ar_msg.header.stamp = ros::Time::now();
    ar_msg.header.frame_id = "camera";
    aruco_info_pub_.publish(ar_msg);
 
    // Show image if no markers are detected
    if (ids.empty()) {
       // ROS_INFO("Markers not found");
       cv::putText(display_image, "no markers found", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
        if (show_detections) {
            //imshow("markers", display_image);
	if (result_img_pub_.getNumSubscribers() > 0)
	{
		result_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_image).toImageMsg());
	}
            auto key = waitKey(1);
            if (key == 27) {
                ROS_INFO("ESC pressed, exit the program");
                ros::shutdown();
            }
        }
       // return;
    }

if(ids.size()>0)
{
    // Compute poses of markers
    vector<Vec3d> rotation_vectors, translation_vectors;
    aruco::estimatePoseSingleMarkers(corners, marker_size, intrinsic_matrix, distortion_coefficients,
                                     rotation_vectors, translation_vectors);
    for (auto i = 0; i < rotation_vectors.size(); ++i) {
        aruco::drawAxis(image, intrinsic_matrix, distortion_coefficients,
                        rotation_vectors[i], translation_vectors[i], marker_size * 0.5f);
    }

    // Draw marker poses
    if (show_detections) {
        aruco::drawDetectedMarkers(display_image, corners, ids);
    }
	if (result_img_pub_.getNumSubscribers() > 0)
	{

                cv::putText(display_image, ""+SSTR(image_width)+"x"+SSTR(image_height)+"@"+SSTR(image_fps)+"FPS m. size: "+SSTR(marker_size)+" m"+" blur: "+SSTR(blur_window_size), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 255, 0), 2);

	for(int i = 0; i<ids.size();i++)
	{
	double prec = getPrec(ids,i);
	if(prec>=min_prec_value)
        {
        Vec3d distance_z_first = translation_vectors[i];
        double distance_z = ROUND3(distance_z_first[2]);
                    cv::putText(display_image, "id: "+SSTR(ids[i])+" z dis: "+SSTR(distance_z)+" m  "+SSTR(ROUND2(prec))+" %", cv::Point(10, 70+i*30), cv::FONT_HERSHEY_SIMPLEX, 0.9, CV_RGB(0, 255, 0), 2);
            result_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_image).toImageMsg());
        }
	}
	}	
        //imshow("markers", display_image); // opencv im_show
        auto key = waitKey(1);
        if (key == 27) {
            ROS_INFO("ESC pressed, exit the program");
            ros::shutdown();
        }


    // Publish TFs for each of the markers
    static tf2_ros::TransformBroadcaster br;
    auto stamp = ros::Time::now();

    // Create and publish tf message for each marker
    tf2_msgs::TFMessage tf_msg_list;
    for (auto i = 0; i < rotation_vectors.size(); ++i)
    {

	    if(getPrec(ids,i)>min_prec_value)
        {
            //ROS_INFO("aruco markers tf");
            auto translation_vector = translation_vectors[i];
            auto rotation_vector = rotation_vectors[i];
            auto transform = create_transform(translation_vector, rotation_vector);
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = frame_id;
            stringstream ss;
            ss << marker_tf_prefix << ids[i];
            tf_msg.child_frame_id = ss.str();
            tf_msg.transform.translation.x = transform.getOrigin().getX();
            tf_msg.transform.translation.y = transform.getOrigin().getY();
            tf_msg.transform.translation.z = transform.getOrigin().getZ();
            tf_msg.transform.rotation.x = transform.getRotation().getX();
            tf_msg.transform.rotation.y = transform.getRotation().getY();
            tf_msg.transform.rotation.z = transform.getRotation().getZ();
            tf_msg.transform.rotation.w = transform.getRotation().getW();
            tf_msg_list.transforms.push_back(tf_msg);
            br.sendTransform(tf_msg);
        }
    }
    tf_list_pub_.publish(tf_msg_list);

}

// rotate vector:
// [ 1 2 3 4 5 ]  --> [ 5 1 2 3 4 ]
if(num_detected>0)
{
//std::cout<<"ids size: vor erase"<<ids.size()<<std::endl;
map<int, vector<int>>::iterator il;
for ( il = ids_hashmap.begin(); il != ids_hashmap.end(); il++ )
{  
 vector<int> current_vector(num_detected);
  current_vector = il->second;
 rotate(current_vector.begin(),current_vector.end()-1,current_vector.end());
il->second = current_vector;
}

// gehe alle in der Liste bestehenden durch:
map<int, vector<int>>::iterator it;
for ( it = ids_hashmap.begin(); it != ids_hashmap.end(); it++ )
{  
  bool current_id_was_found = false;
  for(int j=0;j<ids.size();j++)
  {
   if( (ids[j] == it->first) && (it->second.size()>1))
   {
	current_id_was_found = true;
        ids.erase (ids.begin()+j);
	//std::cout<<"erase "<<ids[j]<<"it first"<<it->first<<"size_second "<<it->second.size()<<std::endl;
   }
  }
  vector<int> current_vector(num_detected);
  current_vector = it->second;
  current_vector[0] = 0;
  if (current_id_was_found)
  {
   current_vector[0] =1;
  //std::cout<<" 1 was set"<<it->first<<std::endl;
  }
it->second = current_vector;
}


// adde alle restlichen in ids (das sind die neu erkannten)
for(int i = 0;i<ids.size();i++)
{

   std::map<int, vector<int>>::iterator ittt = ids_hashmap.begin();
   vector<int> tmpp(num_detected, 0);
   tmpp[0] = 1;
   std::string aa = "";
   for(int i=0;i<num_detected;i++)
	aa+=SSTR(tmpp[i])+",";

   ids_hashmap.insert(make_pair(ids[i], tmpp));
   //std::cout<<"added new: "<<ids[i]<<" "<<aa<<" size tmpp"<<tmpp.size()<<std::endl;
}

//// print the hashmap:
map<int, vector<int>>::iterator itt;
for ( itt = ids_hashmap.begin(); itt != ids_hashmap.end(); itt++ )
{
   vector<int> tmp(num_detected, 0);
   tmp = itt->second; 
// hack -> no idea why this is necessary
if(itt->second.size() ==0)
{
   vector<int> tmpe(num_detected, 0);
   tmpe[0] = 1;
itt->second =tmpe;
} // end of hack
//   std::string a = SSTR(itt->first)+"  "+SSTR(itt->second.size())+ " ";
	


//   for(int i=0;i<tmp.size();i++)
//{
//   a+= SSTR(tmp[i])+",";
//}
//std::cout<<a<<std::endl;
}

} // num_detected>0


}

// TODO: slider extension
// mach ne hashmap von int,array

int main(int argc, char **argv) {
    map<string, aruco::PREDEFINED_DICTIONARY_NAME> dictionary_names;
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_50", aruco::DICT_4X4_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_100", aruco::DICT_4X4_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_250", aruco::DICT_4X4_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_1000", aruco::DICT_4X4_1000));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_50", aruco::DICT_5X5_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_100", aruco::DICT_5X5_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_250", aruco::DICT_5X5_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_1000", aruco::DICT_5X5_1000));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_50", aruco::DICT_6X6_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_100", aruco::DICT_6X6_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_250", aruco::DICT_6X6_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_1000", aruco::DICT_6X6_1000));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_50", aruco::DICT_7X7_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_100", aruco::DICT_7X7_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_250", aruco::DICT_7X7_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_1000", aruco::DICT_7X7_1000));
    dictionary_names.insert(
            pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_ARUCO_ORIGINAL", aruco::DICT_ARUCO_ORIGINAL));

    signal(SIGINT, int_handler);

    // Initalize ROS node
    ros::init(argc, argv, "aruco_detector_ocv");
    ros::NodeHandle nh("~");
    string rgb_topic, rgb_info_topic, dictionary_name;
    nh.param("camera", rgb_topic, string("/kinect2/hd/image_color_rect"));
    nh.param("camera_info", rgb_info_topic, string("/kinect2/hd/camera_info"));
    nh.param("show_detections", show_detections, true);
    nh.param("tf_prefix", marker_tf_prefix, string("marker"));
    nh.param("marker_size", marker_size, 0.09f);
    nh.param("enable_blur", enable_blur, true);
    nh.param("blur_window_size", blur_window_size, 7);
    nh.param("image_fps", image_fps, 30);
    nh.param("image_width", image_width, 640);
    nh.param("image_height", image_height, 480);
    nh.param("num_detected", num_detected, 50);
    nh.param("min_prec_value", min_prec_value, 80);

    detector_params = aruco::DetectorParameters::create();
    detector_params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    nh.param("dictionary_name", dictionary_name, string("DICT_4X4_250"));
    nh.param("aruco_adaptiveThreshWinSizeStep", detector_params->adaptiveThreshWinSizeStep, 4);
    int queue_size = 10;

    // Configure ARUCO marker detector
    dictionary = aruco::getPredefinedDictionary(dictionary_names[dictionary_name]);
    ROS_DEBUG("%f", marker_size);

    if (show_detections) {
       // namedWindow("markers", cv::WINDOW_KEEPRATIO);
    }
    ros::Subscriber rgb_sub = nh.subscribe(rgb_topic.c_str(), queue_size, callback);
    ros::Subscriber rgb_info_sub = nh.subscribe(rgb_info_topic.c_str(), queue_size, callback_camera_info);
    ros::Subscriber parameter_sub = nh.subscribe("/update_params", queue_size, update_params_cb);

    // Publisher:
  image_transport::ImageTransport it(nh);
  result_img_pub_ = it.advertise("/result_img", 1);
  tf_list_pub_    = nh.advertise<tf2_msgs::TFMessage>("/tf_list", 10);

  aruco_info_pub_ = nh.advertise<alfons_msgs::ArucoInfo>("/aruco_list", 10);

    ros::spin();
    return 0;
}
