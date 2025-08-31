// pickplace.cpp

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// MTC pick/place demo implementation
#include <roarm_moveit_mtc_demo/pick_place_task.h>
#include "pick_place_parameters.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <thread>

bool task_ready = false;
bool planning_in_progress = false;
size_t object_idx = 0; // Start from first object

roarm_moveit_mtc_demo::PickPlaceTask* current_task = nullptr;

constexpr int delay_seconds = 2;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("roarm_moveit_mtc_demo");

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("roarm_moveit_mtc_demo", node_options);

    std::thread spinning_thread([node] { rclcpp::spin(node); });

    const auto param_listener = std::make_shared<pick_place_task_demo::ParamListener>(node);
    const auto params = param_listener->get_params();
    roarm_moveit_mtc_demo::setupDemoScene(params);

    auto plan_next_task = [&]() {
        if (object_idx >= params.object_names.size()) {
            RCLCPP_INFO(LOGGER, "All objects picked. Respawning...");

            // Respawn all objects
            auto planning_scene_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            moveit_msgs::msg::PlanningScene planning_scene_msg;
            planning_scene_msg.is_diff = true;

            for (size_t i = 0; i < params.object_names.size(); ++i) {
                moveit_msgs::msg::CollisionObject obj;
                obj.id = params.object_names[i];
                obj.header.frame_id = "world";
                obj.operation = obj.ADD;
                obj.primitives.resize(1);
                obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;

                std::vector<double> dims;
                switch (i) {
                    case 0: dims = params.object_dimensions_1; break;
                    case 1: dims = params.object_dimensions_2; break;
                    case 2: dims = params.object_dimensions_3; break;
                    case 3: dims = params.object_dimensions_4; break;
                    case 4: dims = params.object_dimensions_5; break;
                    case 5: dims = params.object_dimensions_6; break;
                    case 6: dims = params.object_dimensions_7; break;
                    case 7: dims = params.object_dimensions_8; break;
                    // case 8: dims = params.object_dimensions_9; break;
                    // case 9: dims = params.object_dimensions_10; break;
                    default: throw std::runtime_error("Invalid object index.");
                }

                if (dims.size() != 3) {
                RCLCPP_ERROR(LOGGER, "Invalid dimensions for box. Expected 3.");
                continue;
                }

                obj.primitives[0].dimensions.clear();
                for (const auto& d : dims) {
                    obj.primitives[0].dimensions.push_back(d);
                }


                std::vector<double> pose_v;
                switch (i) {
                    case 0: pose_v = params.object_pose_1; break;
                    case 1: pose_v = params.object_pose_2; break;
                    case 2: pose_v = params.object_pose_3; break;
                    case 3: pose_v = params.object_pose_4; break;
                    case 4: pose_v = params.object_pose_5; break;
                    case 5: pose_v = params.object_pose_6; break;
                    case 6: pose_v = params.object_pose_7; break;
                    case 7: pose_v = params.object_pose_8; break;
                    // case 8: pose_v = params.object_pose_9; break;
                    // case 9: pose_v = params.object_pose_10; break;
                    default: throw std::runtime_error("Invalid pose index.");
                }

                geometry_msgs::msg::Pose pose;
                pose.position.x = pose_v[0];
                pose.position.y = pose_v[1];
                pose.position.z = pose_v[2] + 0.5 * dims[2];
                tf2::Quaternion q;
                q.setRPY(pose_v[3], pose_v[4], pose_v[5]);
                pose.orientation = tf2::toMsg(q);

                obj.primitive_poses.push_back(pose);
                planning_scene_msg.world.collision_objects.push_back(obj);
            }

            planning_scene_publisher->publish(planning_scene_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
            object_idx = 0;
        }

        // Plan for the next object
        const std::string& object_id = params.object_names[object_idx];
        RCLCPP_INFO(LOGGER, "Planning pick/place for object: %s", object_id.c_str());

        delete current_task;
        current_task = new roarm_moveit_mtc_demo::PickPlaceTask("pick_place_task");

        if (!current_task->init(node, params, object_id)) {
            RCLCPP_ERROR(LOGGER, "Initialization failed.");
            task_ready = false;
            return;
        }

        if (current_task->plan(params.max_solutions)) {
            RCLCPP_INFO(LOGGER, "Planning complete. Waiting for QR code to execute.");
            task_ready = true;
        } else {
            RCLCPP_ERROR(LOGGER, "Planning failed.");
            task_ready = false;
        }
    };

    // Initial planning before any QR scan
    plan_next_task();

    auto sub = node->create_subscription<std_msgs::msg::String>(
        "pick",
        10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "pick" && task_ready) {
                RCLCPP_INFO(LOGGER, "QR detected. Executing preplanned motion...");
                task_ready = false;

                std::thread exec_thread([&]() {
                    current_task->execute();
                    RCLCPP_INFO(LOGGER, "Execution complete.");

                    // Remove the object
                    const std::string& object_id = params.object_names[object_idx];
                    auto planning_scene_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
                    rclcpp::sleep_for(std::chrono::milliseconds(500));
                    moveit_msgs::msg::PlanningScene planning_scene_msg;
                    planning_scene_msg.is_diff = true;

                    moveit_msgs::msg::CollisionObject obj;
                    obj.id = object_id;
                    obj.header.frame_id = "world";
                    obj.operation = obj.REMOVE;
                    planning_scene_msg.world.collision_objects.push_back(obj);

                    planning_scene_publisher->publish(planning_scene_msg);
                    rclcpp::sleep_for(std::chrono::milliseconds(500));

                    object_idx++;

                    // Re-plan the next task
                    std::this_thread::sleep_for(std::chrono::seconds(delay_seconds));
                    plan_next_task();

                    auto pub = node->create_publisher<std_msgs::msg::String>("resume_scanner", 10);
                    std_msgs::msg::String resume_msg;
                    resume_msg.data = "resume";
                    pub->publish(resume_msg);
                });

                exec_thread.detach();
            }
        }
    );

    spinning_thread.join();
    delete current_task;
    return 0;
}
