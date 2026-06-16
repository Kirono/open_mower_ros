#include <ftc_local_planner/backward_forward_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <angles/angles.h>

namespace ftc_local_planner
{

BackwardForwardRecovery::BackwardForwardRecovery() 
  : initialized_(false),
    max_distance_(0.5),
    linear_vel_(0.6),
    angular_vel_(2.1),
    check_frequency_(10.0),
    max_cost_threshold_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE-10),
    obstacle_check_distance_(0.5),
    timeout_(ros::Duration(10.0)) {}

void BackwardForwardRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* global_costmap,
                                costmap_2d::Costmap2DROS* local_costmap)
{
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name_);
    cmd_vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    private_nh.param("max_distance", max_distance_, 0.5);
    private_nh.param("linear_vel", linear_vel_, 0.6);
    private_nh.param("angular_vel", angular_vel_, 2.1);
    private_nh.param("check_frequency", check_frequency_, 10.0);
    int temp_threshold;
    private_nh.param("max_cost_threshold", temp_threshold, static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE-10));
    max_cost_threshold_ = static_cast<unsigned char>(temp_threshold);
    private_nh.param("obstacle_check_distance", obstacle_check_distance_, 0.5);

    double timeout_seconds;
    private_nh.param("timeout", timeout_seconds, 10.0);
    timeout_ = ros::Duration(timeout_seconds);

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void BackwardForwardRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ROS_WARN("Running Backward/Forward recovery behavior");

	ROS_WARN("trying backwards");
  if (attemptMove(max_distance_, false)) {
    ROS_INFO("Successfully moved backwards");
    return;
  }
  ROS_WARN("trying forwards");
  if (attemptMove(max_distance_, true)) {
    ROS_INFO("Successfully moved forwards");
    return;
  }
  attemptoTurn(max_distance_);
  ROS_WARN("trying forwards");
  if (attemptMove(max_distance_, true)) {
    ROS_INFO("Successfully moved forwards");
    return;
  }
  ROS_WARN("trying backwards half");
  if (attemptMove(max_distance_/2, false)) {
    ROS_INFO("Successfully moved backwards");
    return;
  }
  ROS_WARN("trying forwards half");
  if (attemptMove(max_distance_/2, true)) {
    ROS_INFO("Successfully moved forwards");
    return;
  }
  attemptoTurn(max_distance_/2);
  ROS_WARN("trying forwards half");
  if (attemptMove(max_distance_/2, true)) {
    ROS_INFO("Successfully moved forwards");
    return;
  }

  ROS_WARN("Backward/Forward recovery behavior failed to move in either direction");
}

bool BackwardForwardRecovery::attemptMove(double distance, bool forward)
{
  geometry_msgs::PoseStamped start_pose;
  local_costmap_->getRobotPose(start_pose);

  ros::Rate rate(check_frequency_);
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = forward ? linear_vel_ : -linear_vel_;
  
  geometry_msgs::PoseStamped current_pose;
  local_costmap_->getRobotPose(current_pose);
  
  if (!isDestinationClear(current_pose.pose, forward,distance)) {
			ROS_WARN("Obstacle too close to destination after checking %.2f meters",
					distance);
			cmd_vel.linear.x = 0;
			cmd_vel_pub_.publish(cmd_vel);
			return false;
		}
  
  
  double moved_distance = 0.0;
  ros::Time start_time = ros::Time::now();
  int path_has_been_clear = 0;
  while (moved_distance < distance && (ros::Time::now() - start_time) < timeout_)
  {
    local_costmap_->getRobotPose(current_pose);
    moved_distance = std::hypot(
    current_pose.pose.position.x - start_pose.pose.position.x,
    current_pose.pose.position.y - start_pose.pose.position.y
    );
    bool pathisclear = isPathClear(current_pose.pose, forward);
    if (!pathisclear && path_has_been_clear)
    {
      ROS_WARN("Obstacle too close after moving %.2f meters", moved_distance);
      cmd_vel.linear.x = 0;
      cmd_vel_pub_.publish(cmd_vel);
      return false;
    } else if(pathisclear && !path_has_been_clear){
		ROS_WARN("Obstacle clear after moving %.2f meters",
						moved_distance);
      path_has_been_clear = 1;
    }

    cmd_vel_pub_.publish(cmd_vel);
    rate.sleep();
  }

  cmd_vel.linear.x = 0;
  cmd_vel_pub_.publish(cmd_vel);

  if (moved_distance >= distance) {
    ROS_INFO("%s movement completed successfully", forward ? "Forward" : "Backward");
    return true;
  } else {
    ROS_WARN("%s movement timed out after %.2f seconds", forward ? "Forward" : "Backward", timeout_.toSec());
    return false;
  }
}

bool BackwardForwardRecovery::attemptoTurn(double distance) {
  ROS_INFO("attempts to turn");
  geometry_msgs::PoseStamped start_pose;
  local_costmap_->getRobotPose(start_pose);

  ros::Rate rate(check_frequency_);
  geometry_msgs::Twist cmd_vel;

  double start_yaw = tf2::getYaw(start_pose.pose.orientation);
  ROS_WARN("current start angel %.2f radians", start_yaw);
  double desired_yaw_change = getDesiredYawChange(start_pose.pose , distance);

  geometry_msgs::PoseStamped current_pose;
  local_costmap_->getRobotPose(current_pose);
  double current_yaw  = tf2::getYaw(current_pose.pose.orientation);

  double delta  = angles::shortest_angular_distance(start_yaw, current_yaw);

  cmd_vel.angular.z = (desired_yaw_change >= 0.0) ? fabs(angular_vel_) : -fabs(angular_vel_);
  cmd_vel.linear.x = 0.0;

  ros::Time start_time = ros::Time::now();



  bool succesrotate = 0;
  while ((ros::Time::now() - start_time) < timeout_) {

    local_costmap_->getRobotPose(current_pose);
    current_yaw  = tf2::getYaw(current_pose.pose.orientation);

    delta  = angles::shortest_angular_distance(start_yaw, current_yaw);

    // Check if we've rotated enough
    if (delta  >= desired_yaw_change && desired_yaw_change >= 0) {
      succesrotate = 1;
      break;
    } else if (delta  <= desired_yaw_change && desired_yaw_change <= 0) {
      succesrotate = 1;
      break;
    }


    cmd_vel_pub_.publish(cmd_vel);
    rate.sleep();
  }
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(cmd_vel);

  if (succesrotate) {
    ROS_INFO("successfully turned to %.2f radians",current_yaw);
    return true;
  } else {
    ROS_WARN("trun timed out after truning to %.2f radians",current_yaw);
    return false;
  }
  return false;
}

bool BackwardForwardRecovery::isPathClear(const geometry_msgs::Pose& pose, bool forward)
{
  costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  
  double yaw = tf2::getYaw(pose.orientation);
  if (!forward)
  {
    yaw += M_PI;
  }

  double resolution = costmap->getResolution();
  unsigned int steps = std::ceil(obstacle_check_distance_ / resolution);

  for (unsigned int i = 0; i <= steps; ++i)
  {
    double dist = i * resolution;
    double yaw_cos = std::cos(yaw);
		double yaw_sin = std::sin(yaw);
    double x = pose.position.x + dist * yaw_cos;
    double y = pose.position.y + dist * yaw_sin;

    unsigned int mx, my;
    if (!costmap->worldToMap(x, y, mx, my))
    {
      return false;
    }

    unsigned char cost = costmap->getCost(mx, my);
    if (cost > max_cost_threshold_)
    {
      return false;
    }
    
    std::vector<geometry_msgs::Point> footprint;
    local_costmap_->getOrientedFootprint(footprint);

    for (size_t fp_idx = 0; fp_idx < footprint.size(); fp_idx++) {
      // Calculate the footprint point's position relative to the middle point (x, y)
      double rel_x = footprint[fp_idx].x - x;
      double rel_y = footprint[fp_idx].y - y;

      // Check if this footprint point is in the "front" direction relative to the middle point
      // by computing dot product with the yaw direction vector
      double dot_product = rel_x * yaw_cos + rel_y * yaw_sin;

      // Only check points that are in front of the middle point (positive dot product)
      if (dot_product > 0) {
        double check_x = footprint[fp_idx].x + dist * yaw_cos;
        double check_y = footprint[fp_idx].y + dist * yaw_sin;

        if (!local_costmap_->getCostmap()->worldToMap(check_x, check_y, mx, my)) {
          ROS_WARN("Obstacle too close footprint outside after checking %.2f meters, index %zu", dist, fp_idx);
          return false;
        }
        cost = local_costmap_->getCostmap()->getCost(mx, my);
        if (cost > max_cost_threshold_) {
          ROS_WARN("Obstacle too close footprint above threshold after checking %.2f meters, index %zu", dist, fp_idx);
          return false;
        }
      }
    }
  }

  return true;
}

bool BackwardForwardRecovery::isDestinationClear(const geometry_msgs::Pose& pose, bool forward, double destinationdist)
{
  costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
  
  double yaw = tf2::getYaw(pose.orientation);
  if (!forward)
  {
    yaw += M_PI;
  }
  
  double yaw_cos = std::cos(yaw);
  double yaw_sin = std::sin(yaw);
  double x = pose.position.x + destinationdist * yaw_cos;
  double y = pose.position.y + destinationdist * yaw_sin;

  unsigned int mx, my;
  if (!costmap->worldToMap(x, y, mx, my))
  {
    return false;
  }

  unsigned char cost = costmap->getCost(mx, my);
  if (cost > max_cost_threshold_)
  {
    return false;
  }
  
  std::vector<geometry_msgs::Point> footprint;
  local_costmap_->getOrientedFootprint(footprint);

  for (size_t fp_idx = 0; fp_idx < footprint.size(); fp_idx++) {
    // Calculate the footprint point's position relative to the middle point (x, y)
    double rel_x = footprint[fp_idx].x - x;
    double rel_y = footprint[fp_idx].y - y;

    double check_x = footprint[fp_idx].x + destinationdist * yaw_cos;
    double check_y = footprint[fp_idx].y + destinationdist * yaw_sin;

    if (!local_costmap_->getCostmap()->worldToMap(check_x, check_y, mx, my)) {
      ROS_WARN("Obstacle too close footprint destination outside after checking %.2f meters, index %zu", destinationdist, fp_idx);
      return false;
    }
    cost = local_costmap_->getCostmap()->getCost(mx, my);
    if (cost > max_cost_threshold_) {
      ROS_WARN("Obstacle too close footprint destination above threshold after checking %.2f meters, index %zu", destinationdist, fp_idx);
      return false;
    }
  
  }

  return true;
}
double BackwardForwardRecovery::getDesiredYawChange(const geometry_msgs::Pose &pose, double obstacle_check_path) {
  double yaw = tf2::getYaw(pose.orientation);

  double resolution = local_costmap_->getCostmap()->getResolution();
  unsigned int steps = std::max(1u, (unsigned int)std::ceil((obstacle_check_path * M_PI) / resolution));
	double step_size_angle = M_PI / steps;
	unsigned int max_cost_step = steps / 2;
	unsigned int best_cost = costmap_2d::LETHAL_OBSTACLE;

	std::vector<geometry_msgs::Point> footprint;
  local_costmap_->getOrientedFootprint(footprint);
    
    

    for (unsigned int step = 0; step <= steps; ++step) {

		// Create direction vector for the yaw
		double angle = yaw + (step * step_size_angle)- (M_PI / 2);
		angle = std::fmod(angle, 2.0 * M_PI);
		if (angle < 0){
			angle += 2.0 * M_PI;
		}
		double yaw_cos = std::cos(angle);
		double yaw_sin = std::sin(angle);
        double x = pose.position.x + obstacle_check_path * yaw_cos;
        double y = pose.position.y + obstacle_check_path * yaw_sin;

        unsigned char current_cost = 0;
        unsigned int mx, my;

        if (!local_costmap_->getCostmap()->worldToMap(x, y, mx, my)) {
            current_cost = std::max(current_cost, costmap_2d::LETHAL_OBSTACLE);
        }else{
			current_cost = std::max(current_cost,local_costmap_->getCostmap()->getCost(mx, my));
		}

    for (size_t fp_idx = 0; fp_idx < footprint.size(); fp_idx++) {
      // Calculate the footprint point's position relative to the middle point (x, y)
      double rel_x = footprint[fp_idx].x - x;
      double rel_y = footprint[fp_idx].y - y;

      // Check if this footprint point is in the "front" direction relative to the middle point
      // by computing dot product with the yaw direction vector
      double dot_product = rel_x * yaw_cos + rel_y * yaw_sin;

      // Only check points that are in front of the middle point (positive dot product)
      if (dot_product > 0) {
        double check_x = footprint[fp_idx].x + obstacle_check_path * yaw_cos;
        double check_y = footprint[fp_idx].y + obstacle_check_path * yaw_sin;

        if (!local_costmap_->getCostmap()->worldToMap(check_x, check_y, mx, my)) {
            current_cost = std::max(current_cost,costmap_2d::LETHAL_OBSTACLE);
        } else {
          current_cost = std::max(current_cost,local_costmap_->getCostmap()->getCost(mx, my));
        }
      }
    }
        
		if(current_cost <= best_cost){
			best_cost = current_cost;
			max_cost_step = step;
		}
    }
	ROS_WARN("best turning angle is %.2f radians", (max_cost_step * step_size_angle) - (M_PI / 2));
    return (max_cost_step * step_size_angle)- (M_PI / 2); //return relative angle change
}

}  // namespace ftc_local_planner

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::BackwardForwardRecovery, nav_core::RecoveryBehavior)
