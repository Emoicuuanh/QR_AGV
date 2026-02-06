#ifndef QR_LOCALIZATION_H_
#define QR_LOCALIZATION_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include "json.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_stamped_msgs/StringStamped.h>
#include <std_stamped_msgs/Int16MultiArrayStamped.h>
#include <nav_msgs/Odometry.h>
#include <agv_msgs/DataMatrixStamped.h>

#include <ros/console.h>
#include <iostream>
#include <sstream>
#include <ctime>

using json = nlohmann::json;
namespace qr_code_localization
{
    class QrLocalization
    {
        public:
            QrLocalization(ros::NodeHandle nh);

            ~QrLocalization();

            void sub_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);

            void sub_qr_code_cb(const agv_msgs::DataMatrixStamped::ConstPtr &msg);

            void sub_robot_stt_cb(const std_stamped_msgs::StringStamped::ConstPtr &msg);

            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg);

            double quaternionToRPY(double qx, double qy, double qz, double qw);

            void computeMaptoOdom();

            void robot_pose_cb(const geometry_msgs::Pose::ConstPtr &msg);

        private:
            ros::NodeHandle nh_;

            ros::Subscriber init_pose_;
            ros::Subscriber sub_odom_;
            ros::Subscriber sub_qr_code_;
            ros::Subscriber sub_robot_stt_;
            ros::Subscriber sub_cmd_vel_;
            ros::Subscriber robot_pose;
            ros::Publisher pub_initpose_;
            ros::Publisher m_led_turn_pub_;

            geometry_msgs::Pose odomPose_;
            geometry_msgs::PoseWithCovarianceStamped poseInit_;
            agv_msgs::DataMatrixStamped dataGls621_;
            std_stamped_msgs::Int16MultiArrayStamped led_turn_msg;

            std::string statusRobot_;
            std::string modeRobot_;

            bool newDataQR_;
            bool newDataOdom_;
            bool firstInitPose_;

            double angleRotateCam_;
            double transXCam_;
            double transYCam_;
            double timeGetCmd_;
            double cmdAngularZ_;
            double cmdLinearX_;
            double oldLabelX_;
            double oldLabelY_;
            double oldUpdateMapX_;
            double oldUpdateMapY_;
            double oldUpdateMapYaw_;
            double pose_x, pose_y;
            std::ostringstream oss;

    };
};

#endif