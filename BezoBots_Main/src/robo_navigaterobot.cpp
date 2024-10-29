#include "robo_navigaterobot.hpp"

NavigateRobot::NavigateRobot() : Node("navigate_robot"), initial_pose_set_(false), goal_index_(0),
                                 robot_start_x_(0.0), robot_start_y_(0.0), shelf_x(0.0), shelf_y(0.0),
                                 total_distance_(0.0),
                                 map_width_(500), map_height_(500), scale_(50.0)
{
    // Initialize the action client for the Nav2 goal
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Service client for moving the shelf
    shelf_service_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("set_entity_state");

    // Subscribe to AMCL pose to get localization updates
    amcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&NavigateRobot::amclPoseCallback, this, std::placeholders::_1));

    // Subscribe to odometry data
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&NavigateRobot::odometryCallback, this, std::placeholders::_1));

    // Subscribe to planned path (assuming it's published on "/plan" topic)
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10,
        std::bind(&NavigateRobot::pathCallback, this, std::placeholders::_1));

    // Subscribe to the map topic to get the map image
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&NavigateRobot::mapCallback, this, std::placeholders::_1));

    // Initialize the OpenCV image for path visualization
    map_image_ = cv::Mat::zeros(map_height_, map_width_, CV_8UC3); // Create a black image
    cv::namedWindow("Robot Path Visualization", cv::WINDOW_AUTOSIZE);

    // Print message to notify waiting for 2D Pose Estimate
    RCLCPP_INFO(this->get_logger(), "Waiting for 2D Pose Estimate to be set...");
}

void NavigateRobot::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Update robot's current position
    robot_current_x_ = msg->pose.pose.position.x;
    robot_current_y_ = msg->pose.pose.position.y;

    if (!initial_pose_set_)
    {
        initial_pose_set_ = true;
        robot_start_x_ = robot_current_x_;
        robot_start_y_ = robot_current_y_;
        RCLCPP_INFO(this->get_logger(), "Initial pose has been set. Sending the first goal...");

        // Start by sending the first goal
        sendGoal(0);
    }

    // RCLCPP_INFO(this->get_logger(), "AMCL update received during goal %d", goal_index_);
}

// Map callback to receive and convert the map data to OpenCV image
void NavigateRobot::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;

    // Resize the map_image_ to match the map size
    map_image_ = cv::Mat::zeros(height, width, CV_8UC3);

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = x + y * width;
            int value = msg->data[index];

            if (value == 0)
            {
                // Free space
                map_image_.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // White
            }
            else if (value == 100)
            {
                // Occupied space
                map_image_.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Black
            }
            else
            {
                // Unknown space
                map_image_.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // Gray
            }
        }
    }

    // Show the map image in the OpenCV window
    cv::imshow("Robot Path Visualization", map_image_);
    cv::waitKey(1);
}

// Odometry callback with map and path overlay
void NavigateRobot::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_current_x_ = msg->pose.pose.position.x;
    robot_current_y_ = msg->pose.pose.position.y;

    // Convert robot coordinates to pixel space
    int robot_x_pixel = static_cast<int>(robot_current_x_ * scale_) + map_width_ / 2;
    int robot_y_pixel = static_cast<int>(robot_current_y_ * scale_) + map_height_ / 2;

    // Draw the robot's current position as a green dot
    cv::circle(map_image_, cv::Point(robot_x_pixel, robot_y_pixel), 2, cv::Scalar(0, 255, 0), -1);

    // If the robot is moving toward the second goal, move the shelf accordingly
    // If the robot is moving toward the second goal, start moving the shelf accordingly
    // If the robot is moving toward the second goal, move the shelf accordingly
    if (goal_index_ == 1)
    {
        if (!keep_moving_shelf_)
        {                              // Start the shelf movement thread if not already running
            keep_moving_shelf_ = true; // Set the flag to true

            // Launch a thread that will call moveShelf every 1 second
            shelf_movement_thread_ = std::thread([this]()
                                                 {
                const double target_shelf_x = 3.021488;  // Target X position for the shelf
                const double target_shelf_y = -4.101736;  // Target Y position for the shelf

                while (keep_moving_shelf_) {
                    // Compute the updated shelf position based on robot's current position
                    double shelf_x = robot_current_x_ + 0.05;
                    double shelf_y = robot_current_y_ + 0.05;

                    // Call the moveShelf function
                    moveShelf("Robo_Shelves_11", shelf_x, shelf_y, 0.0343);

                    // Check if the shelf is within the tolerance of its target position
                    if (std::abs(shelf_x - target_shelf_x) <= tolerance && std::abs(shelf_y - target_shelf_y) <= tolerance) {
                        RCLCPP_INFO(this->get_logger(), "Shelf reached target location. Stopping movement.");
                        keep_moving_shelf_ = false;  // Stop moving the shelf
                        break;  // Exit the loop
                    }

                    // Sleep for 1 second before moving the shelf again
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                } });
        }
    }
    else
    {
        // If the goal is not 1, stop moving the shelf
        if (keep_moving_shelf_)
        {
            keep_moving_shelf_ = false; // Stop the shelf movement
            if (shelf_movement_thread_.joinable())
            {
                shelf_movement_thread_.join(); // Join the thread when stopping
            }
        }
    }

    // Show the updated map with the path
    cv::imshow("Robot Path Visualization", map_image_);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Odometry update received during goal %d", goal_index_);
}

void NavigateRobot::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // Draw the planned path on the image
    drawPlannedPath(msg);
}

void NavigateRobot::drawPlannedPath(const nav_msgs::msg::Path::SharedPtr path)
{
    // Draw the planned path as red dots
    for (const auto &pose : path->poses)
    {
        int path_x_pixel = static_cast<int>(pose.pose.position.x * scale_) + map_width_ / 2;
        int path_y_pixel = static_cast<int>(pose.pose.position.y * scale_) + map_height_ / 2;

        // Draw planned path as red dots
        cv::circle(map_image_, cv::Point(path_x_pixel, path_y_pixel), 3, cv::Scalar(0, 0, 255), -1);
    }

    // Display the updated image
    cv::imshow("Robot Path Visualization", map_image_);
    cv::waitKey(1); // Small delay to refresh the image
}

void NavigateRobot::sendGoal(int goal_index)
{
    // Define the two hardcoded goals
    std::vector<std::tuple<double, double, double>> goals = {
        {-0.044, 1.31, 0.0},       // First goal: Center under the shelf
        {3.021488, -4.101736, 0.0} // Second goal: Drop-off location
    };

    // Check if we've exceeded the number of goals
    if (goal_index >= goals.size())
    {
        RCLCPP_INFO(this->get_logger(), "All goals reached.");
        return;
    }

    // Get the current goal coordinates
    auto [goal_x, goal_y, yaw] = goals[goal_index];
    RCLCPP_INFO(this->get_logger(), "Sending goal %d: (x = %f, y = %f, yaw = %f)", goal_index + 1, goal_x, goal_y, yaw);

    // Calculate total distance for the second goal
    if (goal_index == 1)
    {
        total_distance_ = calculateTotalDistanceToGoal(goal_x, goal_y);
        RCLCPP_INFO(this->get_logger(), "Total distance to goal 2: %f meters", total_distance_);
    }

    // Construct the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = goal_x;
    goal_msg.pose.pose.position.y = goal_y;
    goal_msg.pose.pose.orientation.z = sin(yaw / 2.0);
    goal_msg.pose.pose.orientation.w = cos(yaw / 2.0);

    // Set up goal options for feedback and result handling
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // Feedback during goal execution
    send_goal_options.feedback_callback = [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle,
                                                 const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // RCLCPP_INFO(this->get_logger(), "Feedback received: Distance remaining to goal = %f", feedback->distance_remaining);
    };

    // Result callback when the goal is completed
    send_goal_options.result_callback = [this, goal_index](const auto &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Goal %d reached successfully!", goal_index + 1);

            // Send the next goal only when the current goal is finished
            if (goal_index == 0)
            {
                sendGoal(1);      // Send the second goal after the first is completed
                goal_index_ += 1; // Increment to goal_index_
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach goal %d.", goal_index + 1);
        }
    };

    // Send the goal asynchronously
    action_client_->async_send_goal(goal_msg, send_goal_options);
}

double NavigateRobot::calculateTotalDistanceToGoal(double goal_x, double goal_y)
{
    // Calculate the total distance between the robot's start position and the goal
    double total_distance = sqrt(pow(goal_x - robot_start_x_, 2) + pow(goal_y - robot_start_y_, 2));
    return total_distance;
}

void NavigateRobot::moveShelf(const std::string &shelf_name, double x, double y, double z)
{
    // Create a request for the SetEntityState service
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = shelf_name;
    request->state.pose.position.x = x;
    request->state.pose.position.y = y;
    request->state.pose.position.z = z;
    request->state.pose.orientation.w = 1.0; // No rotation
    request->state.reference_frame = "world";

    // Check if the service is available before calling
    if (!shelf_service_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "SetEntityState service not available after waiting");
        return;
    }

    // Send the service request asynchronously
    auto result = shelf_service_client_->async_send_request(request);
    result.wait();

    if (result.get()->success)
    {
        RCLCPP_INFO(this->get_logger(), "Moved shelf %s to position x: %f, y: %f, z: %f", shelf_name.c_str(), x, y, z);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to move shelf %s", shelf_name.c_str());
    }
}
