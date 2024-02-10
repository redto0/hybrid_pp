#include "hybrid_pp/PurePursuitNode_node.hpp"

// For _1
using namespace std::placeholders;

// For 1000ms
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
    : Node("PurePursuitNode", options), rate(this->declare_parameter<float>("frequency", 30)) {
    // Params
    min_look_ahead_distance = this->declare_parameter<float>("min_look_ahead_distance",
                                                             3.85);  // This is set to the min turning radius of phnx
    max_look_ahead_distance = this->declare_parameter<float>("max_look_ahead_distance", 10.0);
    k_dd = this->declare_parameter<float>("k_dd", 1.0);
    max_speed = this->declare_parameter<float>("max_speed", 6.7056);
    rear_axle_frame = this->declare_parameter<std::string>("rear_axle_frame", "rear_axle");
    wheel_base = this->declare_parameter<float>("wheel_base", 1.08);
    gravity_constant = this->declare_parameter<float>("gravity_constant", 9.81);
    debug = this->declare_parameter<bool>("debug", true);
    this->declare_parameter<bool>("stop_on_no_path", true);

    // Var init
    current_speed = 1;

    // Pub Sub
    path_sub =
        this->create_subscription<nav_msgs::msg::Path>("/path", 5, std::bind(&PurePursuitNode::path_cb, this, _1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 5,
                                                                  std::bind(&PurePursuitNode::odom_speed_cb, this, _1));
    lidar_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 5, std::bind(&PurePursuitNode::lidar_scan_cb, this, _1));
    nav_ack_vel_pub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/nav_ack_vel", 5);

    // Visualization marker publishers
    travel_path_pub = this->create_publisher<visualization_msgs::msg::Marker>("/travel_path_marker", 1);
    look_ahead_vis_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/look_ahead_marker", 1);
    intersection_point_pub = this->create_publisher<visualization_msgs::msg::Marker>("/intersection_marker", 1);
    planner_path_pub = this->create_publisher<visualization_msgs::msg::Marker>("/planner_path_marker", 1);
    object_vis_pub = this->create_publisher<visualization_msgs::msg::Marker>("/object_vis_marker", 1);

    // TF
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->work_thread = std::thread{[this]() {
        RCLCPP_INFO(this->get_logger(), "Beginning pure pursuit loop...");
        while (true) {
            // Break if node is dying
            if (this->stop_token.load()) {
                break;
            }

            /// The calculated intersection and look ahead distance
            std::optional<PathCalcResult> path_result;
            /// Visualization compoenents for publishing visualization
            VisualizationComponents vis_components{};

            if (!path_spline.has_value()) {
                continue;
            }

            {
                std::unique_lock lk{this->spline_mtx};
                path_result = this->get_path_point(this->path_spline.value());
            }

            // If no way to follow path, just stop
            if (!path_result.has_value()) {
                if (this->get_parameter("stop_on_no_path").as_bool()) {
                    RCLCPP_INFO(this->get_logger(), "No intersection with path, stopping!");
                    this->publish_stop_command();
                }

                continue;
            }

            // Calculate command to point on path
            auto command =
                this->calculate_command_to_point((*path_result).intersection_point, (*path_result).look_ahead_distance);

            {
                std::unique_lock lk{this->spline_mtx};
                std::unique_lock lk2{this->obj_mtx};

                vis_components.intersection_point = (*path_result).intersection_point;
                vis_components.look_ahead_distance = (*path_result).look_ahead_distance;
                vis_components.objects = this->objects;
                vis_components.path_spline = this->path_spline;
                vis_components.turning_radius = command.turning_radius;

                if (debug) {
                    publish_visualisation(vis_components);
                }
            }

            nav_ack_vel_pub->publish(command.command);

            // Run at some rate
            if (!this->rate.sleep()) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Unable to meet desired frequency! This will only print once.");
            }
        }
    }};
}

void PurePursuitNode::path_cb(nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.at(0).header.frame_id.empty()) {
        RCLCPP_INFO(this->get_logger(), "Path has no frame id");
        return;
    }

    {
        std::unique_lock lk{this->spline_mtx};
        std::unique_lock lk2{this->odom_mtx};
        std::unique_lock lk3{this->obj_mtx};
        this->path_spline = get_path_spline(*msg);
    }
}

CommandCalcResult PurePursuitNode::calculate_command_to_point(const geometry_msgs::msg::PoseStamped& target_point,
                                                              float look_ahead_distance) const {
    // Calculate the angle between the robot and the goal.
    float alpha = atan2f((float)target_point.pose.position.y, (float)target_point.pose.position.x);

    // Set the desired steering angle and set it to the ackerman message
    float steering_angle = atanf(2.0f * wheel_base * sinf(alpha) / look_ahead_distance);
    static ackermann_msgs::msg::AckermannDrive ack_msg{};
    ack_msg.steering_angle = (steering_angle + ack_msg.steering_angle) / 2;

    // Gets the distance to the instantaneous center of rotation from the center of the rear axle (turning radius)
    float distance_to_icr = wheel_base / std::tan(steering_angle);

    // Set the speed based off the eq v = sqrt(static_friction * gravity * turn_radius) with static friction being 1.
    // This finds the fastest speed that can be taken without breaking friction and slipping.
    float set_speed =
        std::clamp(std::sqrt(this->gravity_constant * std::abs(distance_to_icr) * 0.3f), 0.1f, this->max_speed);
    ack_msg.speed = (set_speed + ack_msg.speed) / 2;

    CommandCalcResult out{ack_msg, look_ahead_distance, distance_to_icr};
    return out;
}

void PurePursuitNode::publish_visualisation(VisualizationComponents vis_compoenets) {
    visualization_msgs::msg::Marker path_prediction_marker{};
    // Declares the path prediction marker in the rear axle frame
    path_prediction_marker.header.frame_id = rear_axle_frame;
    path_prediction_marker.header.stamp = get_clock()->now();
    path_prediction_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_prediction_marker.action = visualization_msgs::msg::Marker::ADD;
    path_prediction_marker.ns = "hybrid_pp_ns";
    path_prediction_marker.id = 0;
    path_prediction_marker.pose.orientation.w = 1.0;
    path_prediction_marker.scale.x = 0.1;
    path_prediction_marker.color.a = 1.0;
    path_prediction_marker.color.g = 1.0;

    visualization_msgs::msg::Marker look_ahead_distance_marker{};
    // Declares the look ahead marker in the rear axle frame
    look_ahead_distance_marker.header.frame_id = rear_axle_frame;
    look_ahead_distance_marker.header.stamp = get_clock()->now();
    look_ahead_distance_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    look_ahead_distance_marker.action = visualization_msgs::msg::Marker::ADD;
    look_ahead_distance_marker.ns = "hybrid_pp_ns";
    look_ahead_distance_marker.id = 1;
    look_ahead_distance_marker.pose.orientation.w = 1.0;
    look_ahead_distance_marker.scale.x = 0.1;
    look_ahead_distance_marker.color.a = 1.0;
    look_ahead_distance_marker.color.r = 1.0;

    std::vector<geometry_msgs::msg::Point> vis_spline;
    for (auto i : vis_compoenets.path_spline.value()) {
        vis_spline.push_back(i.pose.position);
    }

    visualization_msgs::msg::Marker spline_marker{};
    spline_marker.header.frame_id = this->rear_axle_frame;
    spline_marker.header.stamp = get_clock()->now();
    spline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    spline_marker.action = visualization_msgs::msg::Marker::ADD;
    spline_marker.ns = "hybrid_pp_ns";
    spline_marker.id = 1;
    spline_marker.pose.orientation.w = 1.0;
    spline_marker.scale.x = 0.1;
    spline_marker.color.a = 1.0;
    spline_marker.color.r = 1.0;
    spline_marker.points = vis_spline;

    visualization_msgs::msg::Marker intersection_marker;
    intersection_marker.header.frame_id = this->rear_axle_frame;
    intersection_marker.header.stamp = get_clock()->now();
    intersection_marker.type = visualization_msgs::msg::Marker::CUBE;
    intersection_marker.action = visualization_msgs::msg::Marker::ADD;
    intersection_marker.ns = "hybrid_pp_ns";
    intersection_marker.id = 1;
    intersection_marker.pose = vis_compoenets.intersection_point.pose;
    intersection_marker.scale.x = 0.5;
    intersection_marker.scale.y = 0.5;
    intersection_marker.scale.z = 0.5;
    intersection_marker.color.a = 1.0;
    intersection_marker.color.r = 0.6;
    intersection_marker.color.b = 0.4;

    std::vector<geometry_msgs::msg::Point> vis_points;

    for (auto pose : vis_compoenets.objects) {
        vis_points.push_back(pose.pose.position);
    }

    visualization_msgs::msg::Marker object_marker;
    object_marker.header.frame_id = this->rear_axle_frame;
    object_marker.header.stamp = get_clock()->now();
    object_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    object_marker.action = visualization_msgs::msg::Marker::ADD;
    object_marker.ns = "hybrid_pp_ns";
    object_marker.id = 1;
    object_marker.points = vis_points;
    object_marker.scale.x = 0.5;
    object_marker.scale.y = 0.5;
    object_marker.scale.z = 0.5;
    object_marker.color.a = 1.0;
    object_marker.color.r = 0.6;
    object_marker.color.b = 0.4;

    // Position used for plotting the visualization markers
    geometry_msgs::msg::Point path_graph_point;
    geometry_msgs::msg::Point look_ahead_graph_point;

    // If our steering angle is negative, graphing math is a little different
    if (vis_compoenets.turning_radius < 0) {
        // Loop runs until the point generated hits the look ahead distance raduis
        for (double index = 0; distance(path_graph_point, zero) - vis_compoenets.look_ahead_distance <= 0;
             index += 0.001) {
            // Sets the x and y coords using polar coodiants the the ICR as the origin
            path_graph_point.x = vis_compoenets.turning_radius * -sin(index);
            path_graph_point.y = vis_compoenets.turning_radius * -cos(index) + vis_compoenets.turning_radius;

            // Appends the generated point to the markers list of points
            path_prediction_marker.points.push_back(path_graph_point);
        }
    } else {
        // Loop runs until the point generated hits the look ahead distance raduis
        for (double index = -3.14 / 2; distance(path_graph_point, zero) - vis_compoenets.look_ahead_distance <= 0;
             index += 0.001) {
            // Sets the x and y coords using ploar coodiants the the ICR as the origin
            path_graph_point.x = vis_compoenets.turning_radius * cos(index);
            path_graph_point.y = vis_compoenets.turning_radius * sin(index) + vis_compoenets.turning_radius;

            // Appends the generated point to the markers list of points
            path_prediction_marker.points.push_back(path_graph_point);
        }
    }

    // Appends a list of points in a circle to the look ahead distance marker using look ahead distance as radius
    for (double index = 0; index <= 2 * 3.14; index += 0.01) {
        look_ahead_graph_point.x = vis_compoenets.look_ahead_distance * cos(index);
        look_ahead_graph_point.y = vis_compoenets.look_ahead_distance * sin(index);

        look_ahead_distance_marker.points.push_back(look_ahead_graph_point);
    }

    // Publish the markers to their respective visualization topics
    planner_path_pub->publish(spline_marker);
    intersection_point_pub->publish(intersection_marker);
    object_vis_pub->publish(object_marker);
    travel_path_pub->publish(path_prediction_marker);
    look_ahead_vis_marker_pub->publish(look_ahead_distance_marker);
}

void PurePursuitNode::odom_speed_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
        std::unique_lock lk{this->odom_mtx};
        current_speed = msg->twist.twist.linear.x;
    }
}

std::optional<PathCalcResult> PurePursuitNode::get_path_point(
    std::vector<geometry_msgs::msg::PoseStamped> path_spline) {
    // Calc look ahead distance
    float look_ahead_distance = std::clamp(k_dd * current_speed, min_look_ahead_distance, max_look_ahead_distance);

    geometry_msgs::msg::PoseStamped intercepted_pose{};

    // Find point on the spline that intersects our lookahead, must be in front of us
    bool point_found = false;
    for (const auto& i : path_spline) {
        // The spline is somewhat sparse, so have some leeway in selecting points on it
        if (i.pose.position.x >= 0 && std::abs(distance(i.pose.position, zero) - look_ahead_distance) <= 1.0) {
            intercepted_pose = i;
            point_found = true;
            break;
        }
    }

    if (!point_found) {
        return std::nullopt;
    }

    PathCalcResult out{};
    out.intersection_point = intercepted_pose;
    out.look_ahead_distance = look_ahead_distance;

    return out;
}
void PurePursuitNode::lidar_scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto laser_scan = msg;

    {
        std::unique_lock lk{this->obj_mtx};
        this->objects = get_objects_from_scan(laser_scan);
    }
}

std::vector<geometry_msgs::msg::PoseStamped> PurePursuitNode::get_objects_from_scan(
    std::shared_ptr<sensor_msgs::msg::LaserScan>& laser_scan) {
    auto trans =
        this->tf_buffer->lookupTransform(this->rear_axle_frame, laser_scan->header.frame_id, tf2::TimePointZero);

    int count = 0;

    std::map<int, int> obj_index_pairs;
    std::vector<geometry_msgs::msg::PoseStamped> detected_objects;

    // needs map for index pairs for objs with consecutive indexs greater than 10
    for (size_t i = 0; i < laser_scan->ranges.size(); i++) {
        int starting_index = i;
        int ending_index = i + 1;
        count = 0;
        for (size_t j = i; j < laser_scan->ranges.size() - i; j++) {
            if (laser_scan->ranges.at(j) < 15) {
                count++;
            } else {
                ending_index = j;
                break;
            }
        }

        if (count >= 1) {
            obj_index_pairs[starting_index] = ending_index;
        }
    }

    for (auto const& [key, val] : obj_index_pairs) {
        for (int i = key; i < val; i++) {
            geometry_msgs::msg::PoseStamped point;
            point.header.frame_id = laser_scan->header.frame_id;
            point.pose.position.x = laser_scan->ranges.at(i) * cos(laser_scan->angle_increment * i + (3 * 3.14 / 4));
            point.pose.position.y = laser_scan->ranges.at(i) * sin(laser_scan->angle_increment * i + (3 * 3.14 / 4));

            tf2::doTransform(point, point, trans);

            detected_objects.push_back(point);
        }
    }

    return detected_objects;
}
std::vector<geometry_msgs::msg::PoseStamped> PurePursuitNode::get_path_spline(const nav_msgs::msg::Path& path) {
    auto trans = this->tf_buffer->lookupTransform(this->rear_axle_frame, path.header.frame_id, tf2::TimePointZero);

    // Create a copy of the path to avoid mutating the main
    auto local_path = path;

    // Transform path from odom or map to rear_axel. This must be done each iteration, else we can only run on new paths
    for (auto& pose : local_path.poses) {
        tf2::doTransform(pose, pose, trans);
        pose.header.frame_id = this->rear_axle_frame;
    }
    local_path.header.frame_id = this->rear_axle_frame;

    // Spline connecting our path points
    std::vector<geometry_msgs::msg::PoseStamped> spline;

    // Build a spline of linear lines for our path
    for (size_t i = 0; i < local_path.poses.size() - 1; i++) {
        auto point1 = local_path.poses.at(i);
        auto point2 = local_path.poses.at(i + 1);

        // Do the DDA algorithm https://en.wikipedia.org/wiki/Digital_differential_analyzer_(graphics_algorithm)
        float dx = (float)point2.pose.position.x - (float)point1.pose.position.x;
        float dy = (float)point2.pose.position.y - (float)point1.pose.position.y;
        float steps = std::max(std::abs(dx), std::abs(dy));
        float x_inc = dx / steps;
        float y_inc = dy / steps;

        for (int j = 0; (float)j < steps; j++) {
            point1.pose.position.x += x_inc;
            point1.pose.position.y += y_inc;
            spline.push_back(point1);
        }
    }

    geometry_msgs::msg::Point left_shifted_point;
    geometry_msgs::msg::Point right_shifted_point;

    for (size_t i = 0; i < spline.size(); i++) {
        left_shifted_point = spline.at(i).pose.position;
        right_shifted_point = spline.at(i).pose.position;

        for (size_t j = 0; j < objects.size(); j++) {
            while (distance(left_shifted_point, objects.at(j).pose.position) <= 2) {
                left_shifted_point.y -= 0.01;
            }
        }
        for (size_t j = 0; j < objects.size(); j++) {
            while (distance(right_shifted_point, objects.at(j).pose.position) <= 2) {
                right_shifted_point.y += 0.01;
            }
        }

        spline.at(i).pose.position = distance(left_shifted_point, spline.at(i).pose.position) <=
                                             distance(right_shifted_point, spline.at(i).pose.position)
                                         ? left_shifted_point
                                         : right_shifted_point;
    }

    return spline;
}