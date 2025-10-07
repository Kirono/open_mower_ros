#include <ftc_local_planner/backward_forward_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

namespace ftc_local_planner {

BackwardForwardRecovery::BackwardForwardRecovery() :
		initialized_(false), max_distance_(0.2), linear_vel_(0.30), check_frequency_(
				10.0), max_cost_threshold_(
				costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 10), obstacle_check_distance_(
				0.2), timeout_(ros::Duration(6.0)), obstacle_footprint_(true) {
}

void BackwardForwardRecovery::initialize(std::string name, tf2_ros::Buffer *tf,
		costmap_2d::Costmap2DROS *global_costmap,
		costmap_2d::Costmap2DROS *local_costmap) {
	if (!initialized_) {
		name_ = name;
		tf_ = tf;
		global_costmap_ = global_costmap;
		local_costmap_ = local_costmap;

		ros::NodeHandle private_nh("~/" + name_);
		cmd_vel_pub_ = private_nh.advertise < geometry_msgs::Twist
				> ("/cmd_vel", 1);

		private_nh.param("max_distance", max_distance_, 0.2);
		private_nh.param("linear_vel", linear_vel_, 0.30);
		private_nh.param("check_frequency", check_frequency_, 10.0);
		int temp_threshold;
		private_nh.param("max_cost_threshold", temp_threshold,
				static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 10));
		max_cost_threshold_ = static_cast<unsigned char>(temp_threshold);
		private_nh.param("obstacle_check_distance", obstacle_check_distance_,
				0.2);

		double timeout_seconds;
		private_nh.param("timeout", timeout_seconds, 6.0);
		timeout_ = ros::Duration(timeout_seconds);

		bool obstacle_footprint_;
		private_nh.param("obstacle_footprint", obstacle_footprint_, true);

		initialized_ = true;
	} else {
		ROS_ERROR(
				"You should not call initialize twice on this object, doing nothing");
	}
}

void BackwardForwardRecovery::runBehavior() {
	if (!initialized_) {
		ROS_ERROR(
				"This object must be initialized before runBehavior is called");
		return;
	}

	ROS_WARN("Running Backward/Forward recovery behavior");

	if (attemptMove(max_distance_, false)) {
		ROS_INFO("Successfully moved backwards");
		return;
	}

	if (attemptMove(max_distance_, true)) {
		ROS_INFO("Successfully moved forwards");
		return;
	}

	ROS_WARN(
			"Backward/Forward recovery behavior failed to move in either direction");
	return;
}

bool BackwardForwardRecovery::attemptMove(double distance, bool forward) {
	geometry_msgs::PoseStamped start_pose;
	local_costmap_->getRobotPose(start_pose);

	ros::Rate rate(check_frequency_);
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = forward ? linear_vel_ : -linear_vel_;

	double moved_distance = 0.0;
	ros::Time start_time = ros::Time::now();
	while (moved_distance < distance
			&& (ros::Time::now() - start_time) < timeout_) {
		geometry_msgs::PoseStamped current_pose;
		local_costmap_->getRobotPose(current_pose);

		moved_distance = std::hypot(
				current_pose.pose.position.x - start_pose.pose.position.x,
				current_pose.pose.position.y - start_pose.pose.position.y);

		if (!isPathClear(current_pose.pose, forward)) {
			ROS_WARN("Obstacle too close after moving %.2f meters",
					moved_distance);
			cmd_vel.linear.x = 0;
			cmd_vel_pub_.publish(cmd_vel);
			return false;
		}
		if (!isPathGlobalClear(current_pose.pose, forward)) {
					ROS_WARN("global Obstacle too close after moving %.2f meters",
							moved_distance);
					cmd_vel.linear.x = 0;
					cmd_vel_pub_.publish(cmd_vel);
					return false;
				}

		cmd_vel_pub_.publish(cmd_vel);
		rate.sleep();
	}

	cmd_vel.linear.x = 0;
	cmd_vel_pub_.publish(cmd_vel);

	if (moved_distance >= distance) {
		ROS_INFO("%s movement completed successfully",
				forward ? "Forward" : "Backward");
		return true;
	} else {
		ROS_WARN("%s movement timed out after %.2f seconds",
				forward ? "Forward" : "Backward", timeout_.toSec());
		return false;
	}
}

bool BackwardForwardRecovery::isPathClear(const geometry_msgs::Pose &pose,
                                        bool forward) {
    double yaw = tf2::getYaw(pose.orientation);
    if (!forward) {
        yaw += M_PI;
    }

    double resolution = local_costmap_->getCostmap()->getResolution();
    unsigned int steps = std::ceil(obstacle_check_distance_ / resolution);

    for (unsigned int step = steps; step <= steps; ++step) {
    	//double dist = step * resolution;
    	double dist = resolution;
		// Create direction vector for the yaw
		double yaw_cos = std::cos(yaw);
		double yaw_sin = std::sin(yaw);
        double x = pose.position.x + dist * yaw_cos;
        double y = pose.position.y + dist * yaw_sin;

        unsigned int mx, my;

        if (!local_costmap_->getCostmap()->worldToMap(x, y, mx, my)) {
            ROS_WARN("Obstacle too close middle outside after checking %.2f meters", dist);
            return false;
        }

        unsigned char cost = local_costmap_->getCostmap()->getCost(mx, my);
        if (cost > max_cost_threshold_) {
            ROS_WARN("Obstacle too close middle above threshold after checking %.2f meters", dist);
            return false;
        }

        /*if (obstacle_footprint_) {
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
        }*/
    }
    return true;
}
bool BackwardForwardRecovery::isPathGlobalClear(const geometry_msgs::Pose &pose,
                                        bool forward) {
    double yaw = tf2::getYaw(pose.orientation);
    if (!forward) {
        yaw += M_PI;
    }

    double resolution = global_costmap_->getCostmap()->getResolution();
    unsigned int steps = std::ceil(obstacle_check_distance_ / resolution);

    for (unsigned int step = steps; step <= steps; ++step) {
    	//double dist = step * resolution;
    	double dist = resolution;
		// Create direction vector for the yaw
		double yaw_cos = std::cos(yaw);
		double yaw_sin = std::sin(yaw);
        double x = pose.position.x + dist * yaw_cos;
        double y = pose.position.y + dist * yaw_sin;

        unsigned int mx, my;

        if (!global_costmap_->getCostmap()->worldToMap(x, y, mx, my)) {
            ROS_WARN("global Obstacle too close middle outside after checking %.2f meters", dist);
            return false;
        }

        unsigned char cost = global_costmap_->getCostmap()->getCost(mx, my);
        if (cost > max_cost_threshold_) {
            ROS_WARN("global Obstacle too close middle above threshold after checking %.2f meters", dist);
            return false;
        }

        /*if (obstacle_footprint_) {
            std::vector<geometry_msgs::Point> footprint;
            global_costmap_->getOrientedFootprint(footprint);

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

                    if (!global_costmap_->getCostmap()->worldToMap(check_x, check_y, mx, my)) {
                        ROS_WARN("global Obstacle too close footprint outside after checking %.2f meters, index %zu", dist, fp_idx);
                        return false;
                    }
                    cost = global_costmap_->getCostmap()->getCost(mx, my);
                    if (cost > max_cost_threshold_) {
                        ROS_WARN("global Obstacle too close footprint above threshold after checking %.2f meters, index %zu", dist, fp_idx);
                        return false;
                    }
                }
            }
        }*/
    }
    return true;
}
}  // namespace ftc_local_planner

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::BackwardForwardRecovery, nav_core::RecoveryBehavior)
