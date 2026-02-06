#include "waypoint_global_planner/waypoint_global_planner.h"
#include <agv_msgs/ArrayWaypoints.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

namespace waypoint_global_planner {

WaypointGlobalPlanner::WaypointGlobalPlanner()
    : costmap_ros_(NULL), initialized_(false), clear_waypoints_(false) {}

WaypointGlobalPlanner::WaypointGlobalPlanner(
    std::string name, costmap_2d::Costmap2DROS *costmap_ros) {
  initialize(name, costmap_ros);
}

WaypointGlobalPlanner::~WaypointGlobalPlanner() {}

void WaypointGlobalPlanner::initialize(std::string name,
                                       costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    // get the costmap
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~" + name);

    // load parameters
    pnh.param("epsilon", epsilon_, 1e-1);
    pnh.param("waypoints_per_meter", waypoints_per_meter_, 20);
    pnh.param("test_rviz", test_rviz_, true);
    pnh.param("tolerance_path", tolerance_path_, 0.03);
    // initialize publishers and subscribers
    waypoint_sub_ = pnh.subscribe(
        "/clicked_point", 100, &WaypointGlobalPlanner::waypointCallback, this);
    external_path_sub_ = pnh.subscribe(
        "external_path", 1, &WaypointGlobalPlanner::externalPathCallback, this);
    waypoint_marker_pub_ =
        pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    goal_pub_ =
        nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);
    client_get_waypoints_ =
        nh.serviceClient<agv_msgs::ArrayWaypoints>("waypoints_to_global");
    client_get_direction_ =
        nh.serviceClient<agv_msgs::DirectionMove>("get_diriction_move");
    initialized_ = true;
    ROS_INFO("Planner has been initialized");
  } else {
    ROS_WARN("This planner has already been initialized");
  }
}

bool WaypointGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped &start_pose,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!test_rviz_) {
    agv_msgs::ArrayWaypoints srv_get_wps;
    agv_msgs::DirectionMove srv_direction;
    client_get_waypoints_.call(srv_get_wps);
    client_get_direction_.call(srv_direction);
    ROS_INFO("Call service get waypoint");
    array_waypoints_ = srv_get_wps.response;
    direction_ = srv_direction.response;
    std::vector<geometry_msgs::PoseStamped> temp_waypoint;
    if (array_waypoints_.Waypoints.size() > 1) {
      use_orient_goal_ = false;
      // for (auto i = 0; i < array_waypoints_.Waypoints.size() - 1; i++)
      // {
      //   temp_waypoint.push_back(array_waypoints_.Waypoints[i].pose);
      // }
      temp_waypoint = array_waypoints_.Waypoints;
    } else {
      use_orient_goal_ = true;
      temp_waypoint.push_back(start_pose);
      temp_waypoint.push_back(goal);
    }
    path_.poses = temp_waypoint;
  } else {
    use_orient_goal_ = true;
    path_.poses.insert(path_.poses.begin(), start_pose);
  }

  if (!use_orient_goal_) {
    tf::Quaternion quat(
        start_pose.pose.orientation.x, start_pose.pose.orientation.y,
        start_pose.pose.orientation.z, start_pose.pose.orientation.w);
    double orient_robot_begin = quaternionToEuler(quat);

    double orient_path = atan2(
        (path_.poses[1].pose.position.y - path_.poses[0].pose.position.y),
        (path_.poses[1].pose.position.x - path_.poses[0].pose.position.x));
    double diff_angle = GetNormaliceAngle(orient_path - orient_robot_begin);
    ROS_WARN_STREAM("ORIENT ROBOT" << orient_robot_begin);
    ROS_WARN_STREAM("ORIENT PATH" << orient_path);
    ROS_WARN_STREAM("DIFF ANGLE ROBOT TO PATH" << diff_angle);
    if (direction_.direction == 0) {
      if (abs(diff_angle) > M_PI / 2 + 0.5) {
        revert_orient_ = true;
      } else {
        revert_orient_ = false;
      }
    } else if (direction_.direction == 1) {
      revert_orient_ = false;
    } else {
      revert_orient_ = true;
    }
  } else {
    tf::Quaternion quat(
        start_pose.pose.orientation.x, start_pose.pose.orientation.y,
        start_pose.pose.orientation.z, start_pose.pose.orientation.w);
    double orient_robot_begin = quaternionToEuler(quat);

    double orient_path =
        atan2((path_.poses[1].pose.position.y - start_pose.pose.position.y),
              (path_.poses[1].pose.position.x - start_pose.pose.position.x));
    double diff_angle = GetNormaliceAngle(orient_path - orient_robot_begin);
    ROS_WARN_STREAM("ORIENT ROBOT" << orient_robot_begin);
    ROS_WARN_STREAM("ORIENT PATH" << orient_path);
    ROS_WARN_STREAM("DIFF ANGLE ROBOT TO PATH" << diff_angle);
    if (direction_.direction == 0) {
      if (abs(diff_angle) > M_PI / 2 + 0.5) {
        revert_orient_ = true;
      } else {
        revert_orient_ = false;
      }
    } else if (direction_.direction == 1) {
      revert_orient_ = false;
    } else {
      revert_orient_ = true;
    }
  }
  interpolatePath(path_);
  path_.header.stamp = ros::Time::now();
  path_.header.frame_id = "map";
  plan_pub_.publish(path_);
  plan = path_.poses;
  ROS_INFO("Published global plan");
  return true;
}

void WaypointGlobalPlanner::waypointCallback(
    const geometry_msgs::PointStamped::ConstPtr &waypoint) {
  if (clear_waypoints_) {
    waypoints_.clear();
    clear_waypoints_ = false;
  }

  // add waypoint to the waypoint vector
  waypoints_.push_back(geometry_msgs::PoseStamped());
  waypoints_.back().header = waypoint->header;
  waypoints_.back().pose.position = waypoint->point;
  waypoints_.back().pose.orientation.w = 1.0;

  // create and publish markers
  createAndPublishMarkersFromPath(waypoints_);

  if (waypoints_.size() < 2)
    return;

  geometry_msgs::Pose *p1 = &(waypoints_.end() - 2)->pose;
  geometry_msgs::Pose *p2 = &(waypoints_.end() - 1)->pose;

  // calculate orientation of waypoints
  double yaw =
      atan2(p2->position.y - p1->position.y, p2->position.x - p1->position.x);
  p1->orientation = tf::createQuaternionMsgFromYaw(yaw);

  // calculate distance between latest two waypoints and check if it surpasses
  // the threshold epsilon
  double dist =
      hypot(p1->position.x - p2->position.x, p1->position.y - p2->position.y);
  if (dist < epsilon_) {
    p2->orientation = p1->orientation;
    path_.header = waypoint->header;
    path_.poses.clear();
    path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
    goal_pub_.publish(waypoints_.back());
    clear_waypoints_ = true;
    ROS_INFO("Published goal pose");
  }
}

void WaypointGlobalPlanner::interpolatePath(nav_msgs::Path &path) {
  std::vector<geometry_msgs::PoseStamped> temp_path;
  for (int i = 0; i < static_cast<int>(path.poses.size() - 1); i++) {
    // calculate distance between two consecutive waypoints
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i + 1].pose.position.x;
    double y2 = path.poses[i + 1].pose.position.y;
    double dist = hypot(x1 - x2, y1 - y2);
    int num_wpts = dist * waypoints_per_meter_;
    // calculate orientation of waypoints
    if (!revert_orient_) {
      orient_end_wp_ = atan2(y2 - y1, x2 - x1);
    } else {
      orient_end_wp_ = atan2(y1 - y2, x1 - x2);
    }

    orient_two_last_wp_ = atan2(y2 - y1, x2 - x1);
    path.poses[i].pose.orientation =
        tf::createQuaternionMsgFromYaw(orient_two_last_wp_);

    temp_path.push_back(path.poses[i]);
    geometry_msgs::PoseStamped p = path.poses[i];

    for (int j = 0; j < num_wpts - 2; j++) {
      p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
      p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
      temp_path.push_back(p);
    }
  }
  // update sequence of poses
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);
  if (!use_orient_goal_) {
    if (revert_orient_ && !temp_path.empty()) {
      temp_path.front().pose.orientation.x = 1;
    }
    geometry_msgs::PoseStamped pose_end;
    path.poses.back().pose.orientation =
        path.poses[path.poses.size() - 2].pose.orientation;
    pose_end = path.poses.back();
    pose_end.pose.position.x =
        pose_end.pose.position.x + tolerance_path_ * cos(orient_two_last_wp_);
    pose_end.pose.position.y =
        pose_end.pose.position.y + tolerance_path_ * sin(orient_two_last_wp_);
    pose_end.pose.orientation = tf::createQuaternionMsgFromYaw(orient_end_wp_);
    temp_path.push_back(path.poses.back());
    temp_path.push_back(pose_end);
  } else {
    if (revert_orient_ && !temp_path.empty()) {
      temp_path.front().pose.orientation.x = 1;
    }
    temp_path.push_back(path.poses.back());
  }
  path.poses = temp_path;
}

void WaypointGlobalPlanner::externalPathCallback(
    const nav_msgs::PathConstPtr &plan) {
  path_.poses.clear();
  clear_waypoints_ = true;
  path_.header = plan->header;
  path_.poses = plan->poses;
  createAndPublishMarkersFromPath(path_.poses);
  goal_pub_.publish(path_.poses.back());
}

void WaypointGlobalPlanner::createAndPublishMarkersFromPath(
    const std::vector<geometry_msgs::PoseStamped> &path) {
  // clear previous markers
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header = path[0].header;
  marker.ns = "/move_base/waypoint_global_planner";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.id = 0;
  markers.markers.push_back(marker);
  waypoint_marker_pub_.publish(markers);
  marker.action = visualization_msgs::Marker::ADD;
  markers.markers.clear();

  for (size_t i = 0; i < path.size(); i++) {
    marker.id = i;
    marker.pose.position = path[i].pose.position;
    markers.markers.push_back(marker);
  }

  waypoint_marker_pub_.publish(markers);
}
double WaypointGlobalPlanner::GetNormaliceAngle(double angle) {
  // ---------------------------------------------------------------------
  // Normalize an angle to [-pi, pi].
  // :param angle: (double)
  // :return: (double) double in radian in [-pi, pi]
  // ---------------------------------------------------------------------
  if (angle > M_PI)
    angle -= 2 * M_PI;
  if (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}
double WaypointGlobalPlanner::quaternionToEuler(tf::Quaternion quat_tf) {
  // ---------------------------------------------------------------------
  // Convert quaterniton to euler angle.
  // :param quat_tf: (tf::Quaternion)
  // :return: (double) yaw in radian
  // ---------------------------------------------------------------------
  double roll, pitch, yaw;
  tf::Matrix3x3 m(quat_tf);
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

} // namespace waypoint_global_planner
