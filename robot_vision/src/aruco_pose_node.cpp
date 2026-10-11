#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <mutex>
#include <string>
#include <vector>
#include <map>

namespace {

cv::Ptr<cv::aruco::Dictionary> getDictionary(const std::string& name) {
  static const std::map<std::string, int> dict_map = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
  };

  auto it = dict_map.find(name);
  if (it == dict_map.end()) {
    ROS_WARN_STREAM("Unknown dictionary '" << name << "'. Falling back to DICT_6X6_250");
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  }
  return cv::aruco::getPredefinedDictionary(it->second);
}

bool getCameraMatrixAndDistortion(const sensor_msgs::CameraInfo& info,
                                  cv::Mat& camera_matrix,
                                  cv::Mat& dist_coeffs) {
  camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  const auto& K = info.K;
  camera_matrix.at<double>(0, 0) = K[0];
  camera_matrix.at<double>(0, 1) = K[1];
  camera_matrix.at<double>(0, 2) = K[2];
  camera_matrix.at<double>(1, 0) = K[3];
  camera_matrix.at<double>(1, 1) = K[4];
  camera_matrix.at<double>(1, 2) = K[5];
  camera_matrix.at<double>(2, 0) = K[6];
  camera_matrix.at<double>(2, 1) = K[7];
  camera_matrix.at<double>(2, 2) = K[8];

  const size_t num_dist = info.D.size();
  if (num_dist == 0) {
    dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  } else {
    dist_coeffs = cv::Mat(1, std::max<size_t>(5, num_dist), CV_64F, cv::Scalar(0.0));
    for (size_t i = 0; i < num_dist; ++i) {
      dist_coeffs.at<double>(0, i) = info.D[i];
    }
  }

  return true;
}

void poseToQuaternion(const cv::Vec3d& rvec,
                      const cv::Vec3d& tvec,
                      geometry_msgs::Pose& pose) {
  cv::Mat rot_matrix;
  cv::Rodrigues(rvec, rot_matrix);

  tf2::Matrix3x3 matrix(
    rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2),
    rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2),
    rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2));

  tf2::Quaternion quaternion;
  matrix.getRotation(quaternion);

  pose.position.x = tvec[0];
  pose.position.y = tvec[1];
  pose.position.z = tvec[2];
  pose.orientation = tf2::toMsg(quaternion);
}

}  // namespace

class ArucoPoseNode {
public:
  ArucoPoseNode() : it_(nh_), tf_broadcaster_() {
    private_nh_ = ros::NodeHandle("~");

    std::string image_topic = "/camera/image_raw";
    std::string camera_info_topic = "/camera/camera_info";
    std::string camera_frame = "camera";
    std::string dictionary_name = "DICT_6X6_250";

    private_nh_.param("image_topic", image_topic, image_topic);
    private_nh_.param("camera_info_topic", camera_info_topic, camera_info_topic);
    private_nh_.param("camera_frame", camera_frame, camera_frame);
    private_nh_.param("marker_size", marker_size_, 0.08);
    private_nh_.param("dictionary", dictionary_name, dictionary_name);

    dictionary_ = getDictionary(dictionary_name);
    image_sub_ = it_.subscribe(image_topic, 1, &ArucoPoseNode::imageCallback, this);
    info_sub_ = nh_.subscribe(camera_info_topic, 1, &ArucoPoseNode::cameraInfoCallback, this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco_pose", 10);
    pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/aruco_pose_array", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/aruco_markers", 10);

    camera_frame_ = camera_frame;
    ROS_INFO_STREAM("Aruco pose node started. Image topic: " << image_topic
                    << ", camera info topic: " << camera_info_topic
                    << ", marker size: " << marker_size_ << " m");
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    std::lock_guard<std::mutex> lock(info_mutex_);
    camera_info_ = *msg;
    have_camera_info_ = true;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    sensor_msgs::CameraInfo info;
    {
      std::lock_guard<std::mutex> lock(info_mutex_);
      if (!have_camera_info_) {
        ROS_WARN_THROTTLE(5.0, "No camera_info received yet. Waiting for camera calibration.");
        return;
      }
      info = camera_info_;
    }

    try {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      if (image.empty()) {
        ROS_WARN_THROTTLE(5.0, "Empty image.");
        return;
      }

      cv::Mat camera_matrix;
      cv::Mat dist_coeffs;
      getCameraMatrixAndDistortion(info, camera_matrix, dist_coeffs);

      std::vector<std::vector<cv::Point2f>> corners, rejected;
      std::vector<int> ids;

      cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
      parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
      cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters, rejected);

      if (ids.empty()) {
        return;
      }

      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix, dist_coeffs, rvecs, tvecs);

      geometry_msgs::PoseArray pose_array;
      pose_array.header = msg->header;
      pose_array.header.frame_id = camera_frame_.empty() ? info.header.frame_id.empty() ? "camera" : info.header.frame_id : camera_frame_;

      visualization_msgs::MarkerArray markers;
      markers.markers.reserve(ids.size());

      for (size_t i = 0; i < ids.size(); ++i) {
        geometry_msgs::Pose pose;
        poseToQuaternion(rvecs[i], tvecs[i], pose);

        pose_array.poses.push_back(pose);

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = pose_array.header.frame_id;
        pose_msg.pose = pose;
        pose_pub_.publish(pose_msg);

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header = msg->header;
        tf_msg.header.frame_id = pose_array.header.frame_id;
        tf_msg.child_frame_id = "aruco_marker_" + std::to_string(ids[i]);
        tf_msg.transform.translation.x = tvecs[i][0];
        tf_msg.transform.translation.y = tvecs[i][1];
        tf_msg.transform.translation.z = tvecs[i][2];
        tf_msg.transform.rotation = pose.orientation;
        tf_broadcaster_.sendTransform(tf_msg);

        visualization_msgs::Marker marker;
        marker.header = msg->header;
        marker.header.frame_id = pose_array.header.frame_id;
        marker.ns = "aruco_markers";
        marker.id = ids[i];
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = marker_size_ * 0.8;
        marker.scale.y = marker_size_ * 0.8;
        marker.scale.z = 0.02;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;
        marker.lifetime = ros::Duration(0.5);
        markers.markers.push_back(marker);
      }

      pose_array_pub_.publish(pose_array);
      marker_pub_.publish(markers);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch (const cv::Exception& e) {
      ROS_ERROR("OpenCV exception: %s", e.what());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber info_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher pose_array_pub_;
  ros::Publisher marker_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  double marker_size_ = 0.08;
  std::string camera_frame_;

  sensor_msgs::CameraInfo camera_info_;
  bool have_camera_info_ = false;
  std::mutex info_mutex_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_pose_node");
  ArucoPoseNode node;
  ros::spin();
  return 0;
}
