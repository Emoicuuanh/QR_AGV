#include <qr_localization/json.hpp>
#include <qr_localization/qr_code_localization.h>

namespace qr_code_localization {
using json = nlohmann::json;
QrLocalization::QrLocalization(ros::NodeHandle nh) : nh_(nh) {
  sub_odom_ = nh_.subscribe("/odom", 1, &QrLocalization::sub_odom_cb, this);
  sub_qr_code_ =
      nh_.subscribe("/data_gls621", 1, &QrLocalization::sub_qr_code_cb, this);
  sub_robot_stt_ = nh_.subscribe("/robot_status", 1,
                                 &QrLocalization::sub_robot_stt_cb, this);
  sub_cmd_vel_ =
      nh_.subscribe("/cmd_vel", 1, &QrLocalization::cmdVelCallback, this);
  robot_pose =
      nh_.subscribe("/rosbot_pose", 1, &QrLocalization::robot_pose_cb, this);

  pub_initpose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 10);
  m_led_turn_pub_ = nh_.advertise<std_stamped_msgs::Int16MultiArrayStamped>(
      "/arduino_driver/led_turn", 1);

  angleRotateCam_ = nh.param<double>("angle_cam_qr", 0.05);
  transXCam_ = nh.param<double>("transform_x_cam_qr", 0.0);
  transYCam_ = nh.param<double>("transform_y_cam_qr", 0.0);
  newDataQR_ = false;
  newDataOdom_ = false;
  firstInitPose_ = true;
  led_turn_msg.data.resize(4);
  cmdAngularZ_ = 0.0;
}

QrLocalization::~QrLocalization() {}

void QrLocalization::cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg) {
  timeGetCmd_ = ros::Time::now().toSec();
  cmdAngularZ_ = msg->angular.z;
  cmdLinearX_ = msg->linear.x;
}

void QrLocalization::sub_odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  odomPose_ = msg->pose.pose;
}

void QrLocalization::sub_qr_code_cb(
    const agv_msgs::DataMatrixStamped::ConstPtr &msg) {
  dataGls621_ = *msg;
  newDataQR_ = true;
}

void QrLocalization::sub_robot_stt_cb(
    const std_stamped_msgs::StringStamped::ConstPtr &msg) {
  std::string data = msg->data;
  try {
    json jsonData = json::parse(data);
    statusRobot_ = jsonData["status"];
    modeRobot_ = jsonData["mode"];
    if (statusRobot_ == "WAITING_INIT_POSE") {
      firstInitPose_ = true;
    }
  } catch (...) {
    std::cerr << "Lỗi chuyển đổi JSON: " << std::endl;
  }
}

void QrLocalization::robot_pose_cb(const geometry_msgs::Pose::ConstPtr &msg) {
  pose_x = msg->position.x;
  pose_y = msg->position.y;
}
double QrLocalization::quaternionToRPY(double qx, double qy, double qz,
                                       double qw) {
  // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf2::Quaternion quat;
  quat.setX(qx);
  quat.setY(qy);
  quat.setZ(qz);
  quat.setW(qw);

  // The tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

void QrLocalization::computeMaptoOdom() {
  double label_x, label_y, deviation_origin_x, deviation_origin_y,
      deviation_theta, yawOdomtoBaselink, yawMaptoOdom;
  double time_update_QR = 0.0;
  bool pass_update = false;

  // set Rate for loop
  ros::Rate rate(50);

  // Tạo ma trận biến đổi từ transform
  // transOdomtoBaselink : odom -> base_link
  // transBaselinktoQR: base_link -> QR
  // transQRtoMap: QR -> Map
  // transOdomtoMap: odom -> Map
  Eigen::Isometry2d transOdomtoBaselink = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transBaselinktoQrCam = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d qrCamtoQR = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transQRtoMap = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transOdomtoMap = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d transMaptoOdom = Eigen::Isometry2d::Identity();

  Eigen::Vector2d translation;
  Eigen::Rotation2Dd rotation;

  // Chuyển đổi ma trận Eigen thành TransformStamped ROS
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;

  tf2::Quaternion quaternion;
  tf2_ros::TransformBroadcaster br;

  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "odom";

  while (ros::ok()) {
    double time = ros::Time::now().toSec();
    ros::spinOnce();
    if (time - timeGetCmd_ > 0.5) {
      cmdAngularZ_ = 0.0;
      cmdLinearX_ = 0.0;
    }
    if (newDataQR_) {
      // data of Gls621
      label_x = double(dataGls621_.lable.x) / 1000;
      label_y = double(dataGls621_.lable.y) / 1000;
      deviation_origin_x = double(dataGls621_.possition.x) / 1000;
      deviation_origin_y = double(dataGls621_.possition.y) / 1000;
      deviation_theta = dataGls621_.possition.angle;

      if (statusRobot_ == "RUNNING" && modeRobot_ == "AUTO" &&
          abs(cmdLinearX_) > 0.2 && label_x == oldLabelX_ &&
          label_y == oldLabelY_) {
        pass_update = true;
      } else {
        pass_update = false;
        oldLabelX_ = label_x;
        oldLabelY_ = label_y;
      }
      if (!pass_update) {
        // odom -> base_link
        transOdomtoBaselink.translation() << odomPose_.position.x,
            odomPose_.position.y;
        yawOdomtoBaselink =
            quaternionToRPY(odomPose_.orientation.x, odomPose_.orientation.y,
                            odomPose_.orientation.z, odomPose_.orientation.w);
        transOdomtoBaselink.linear() =
            Eigen::Rotation2Dd(yawOdomtoBaselink).toRotationMatrix();

        // base_link -> QR Cam
        transBaselinktoQrCam.translation() << transXCam_, transYCam_;
        transBaselinktoQrCam.linear() =
            Eigen::Rotation2Dd(angleRotateCam_).toRotationMatrix();

        // QRCam -> QR
        qrCamtoQR.translation() << deviation_origin_x, deviation_origin_y;
        qrCamtoQR.linear() =
            Eigen::Rotation2Dd(-deviation_theta).toRotationMatrix();

        // QR -> Map
        transQRtoMap.translation() << -label_x, -label_y;

        // Nhân ma trận để tạo ma trận biến đổi cho odom to Map
        transOdomtoMap = transOdomtoBaselink * transBaselinktoQrCam *
                         qrCamtoQR * transQRtoMap;

        // Tinh ma tran Map chuyen map -> odom
        transMaptoOdom = transOdomtoMap.inverse();

        // Lấy giá trị translation và quaternion từ ma trận Eigen
        translation = transMaptoOdom.translation();
        rotation = Eigen::Rotation2Dd(transMaptoOdom.linear());
        yawMaptoOdom = rotation.angle();
        // send transform map -> odom
        transformStamped.transform.translation.x = translation.x();
        transformStamped.transform.translation.y = translation.y();
        transformStamped.transform.translation.z = 0.0;
        quaternion.setRPY(0, 0, yawMaptoOdom);

        // if(abs(cmdLinearX_) > 0.2)
        // {
        // std::cout << "label_x: " << label_x << std::endl;
        // std::cout << "label_y: " << label_y << std::endl;
        // std::cout << "deviation_origin_x: " << deviation_origin_x <<
        // std::endl; std::cout << "deviation_origin_y: " << deviation_origin_y
        // << std::endl; std::cout << "deviation_theta: " << deviation_theta <<
        // std::endl; std::cout << "update_diff_x: " << translation.x() -
        // oldUpdateMapX_ << std::endl; std::cout << "update_diff_y: " <<
        // translation.y() - oldUpdateMapY_ << std::endl; std::cout <<
        // "update_diff_yaw: " << yawMaptoOdom - oldUpdateMapYaw_ << std::endl;
        // std::cout << "---------------------------------" << std::endl;

        // }
        if (abs(cmdLinearX_) > 0.03 || abs(cmdAngularZ_) > 0.03) {
          // ros::Time(0);
          // Get current time
          std::time_t t = std::time(nullptr);
          std::tm *now = std::localtime(&t);
          oss << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-'
              << now->tm_mday << ' ' << now->tm_hour << ':' << now->tm_min
              << ':' << now->tm_sec;
          double current_time = ros::Time::now().toSec();
          ROS_INFO("TIME: %f", current_time);
          ROS_INFO_STREAM("Custom timestamp: " << oss.str());
          // std::cout << oss.str();
          ROS_INFO("label_x: %f", label_x);
          ROS_INFO("label_y: %f", label_y);
          ROS_INFO("deviation_origin_x: %f", deviation_origin_x);
          ROS_INFO("deviation_origin_y: %f", deviation_origin_y);
          ROS_INFO("deviation_theta: %f", deviation_theta);
          ROS_INFO("robot_x %f", pose_x);
          ROS_INFO("robot_y %f", pose_y);

          // Clear the stream
          oss.str("");
          oss.clear();
        }

        oldUpdateMapX_ = translation.x();
        oldUpdateMapY_ = translation.y();
        oldUpdateMapYaw_ = yawMaptoOdom;

        transformStamped.transform.rotation = tf2::toMsg(quaternion);
        if (firstInitPose_) {
          poseInit_.header.frame_id = "map";
          poseInit_.pose.pose.position.x = 0.0;
          poseInit_.pose.pose.position.y = 0.0;
          poseInit_.pose.pose.orientation.z = 0.0;
          poseInit_.pose.pose.orientation.w = 1.0;
          pub_initpose_.publish(poseInit_);
          firstInitPose_ = false;
        }
        time_update_QR = ros::Time::now().toSec();
      }
      newDataQR_ = false;
    }
    newDataOdom_ = false;
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);
    if ((ros::Time::now().toSec() - time_update_QR) > 0.5) {
      led_turn_msg.data[0] = 0;
      led_turn_msg.data[1] = 0;
      led_turn_msg.data[2] = 0;
      led_turn_msg.data[3] = 0;
      m_led_turn_pub_.publish(led_turn_msg);
    } else {
      led_turn_msg.data[0] = 1;
      led_turn_msg.data[1] = 1;
      led_turn_msg.data[2] = 1;
      led_turn_msg.data[3] = 1;
      m_led_turn_pub_.publish(led_turn_msg);
    }
    rate.sleep();
  }
}
} // namespace qr_code_localization

int main(int argc, char **argv) {
  ros::init(argc, argv, "qr_localization");
  ros::NodeHandle nh("~");
  qr_code_localization::QrLocalization qr(nh);
  qr.computeMaptoOdom();
}