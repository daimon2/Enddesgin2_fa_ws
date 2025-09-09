#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <iostream>
#include <thread>

static char read_key(const std::string& prompt)
{
  std::cout << "\n" << prompt
            << " [p=plan, e=execute(async), c=cancel, r=replan, y=+10mm(local Y), Y=-10mm(local Y), q=quit] > "
            << std::flush;
  char ch = 0;
  std::cin >> ch;
  return ch;
}

// 由四元数得到在规划坐标系中的局部Y轴（Local Y-axis in planning frame）
static geometry_msgs::msg::Vector3 local_y_axis_in_planning(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tq;
  tf2::fromMsg(q, tq);
  tf2::Matrix3x3 R(tq);
  tf2::Vector3 y = R.getColumn(1); // 第二列=局部Y
  geometry_msgs::msg::Vector3 v;
  v.x = y.getX(); v.y = y.getY(); v.z = y.getZ();
  return v;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hello_moveit");

  // 参数（可 --ros-args 覆盖）
  const std::string planning_group = node->declare_parameter<std::string>("planning_group", "arm");
  const std::string eef_link       = node->declare_parameter<std::string>("eef_link", "");
  const std::string tf_source      = node->declare_parameter<std::string>("tf_source_frame", "slicer_line_goal");
  const std::string planning_frame = node->declare_parameter<std::string>("planning_frame", "base_link");

  // 让节点活着
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thr([&exec]() { exec.spin(); });

  // TF
  tf2_ros::Buffer tf_buffer{node->get_clock()};
  tf2_ros::TransformListener tf_listener{tf_buffer};

  // MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  if (!eef_link.empty()) move_group.setEndEffectorLink(eef_link);
  move_group.setPoseReferenceFrame(planning_frame);
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // 初始 TF -> 目标位姿
  geometry_msgs::msg::TransformStamped tf_msg;
  const auto t0 = node->now();
  while (rclcpp::ok()) {
    try {
      tf_msg = tf_buffer.lookupTransform(planning_frame, tf_source, tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException& ex) {
      if ((node->now() - t0).seconds() > 3.0) {
        RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
        rclcpp::shutdown(); spin_thr.join(); return 1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = planning_frame;
  goal.header.stamp    = node->get_clock()->now();
  goal.pose.position.x = tf_msg.transform.translation.x;
  goal.pose.position.y = tf_msg.transform.translation.y;
  goal.pose.position.z = tf_msg.transform.translation.z;
  goal.pose.orientation = tf_msg.transform.rotation;

  RCLCPP_INFO(node->get_logger(),
              "Target from TF '%s' in '%s': (%.3f, %.3f, %.3f)",
              tf_source.c_str(), planning_frame.c_str(),
              goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool has_plan = false;

  while (rclcpp::ok()) {
    const char k = read_key("请选择操作");
    if (k == 'q' || k == 'Q') break;

    if (k == 'p' || k == 'P') {
      // 仅规划
      move_group.clearPoseTargets();
      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(goal);
      auto ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      has_plan = ok;
      RCLCPP_INFO(node->get_logger(), "[plan] %s", ok ? "SUCCESS" : "FAILED");

    } else if (k == 'e' || k == 'E') {
      // 异步执行 —— 发送后立即返回菜单（可继续 cancel/replan）
      if (!has_plan) {
        RCLCPP_WARN(node->get_logger(), "No plan yet. Press 'p' to plan first.");
        continue;
      }
      auto rc = move_group.asyncExecute(plan.trajectory_);
      RCLCPP_INFO(node->get_logger(), "[execute-async] sent, code=%d (returning to menu)", rc.val);

    } else if (k == 'c' || k == 'C') {
      // 取消当前执行
      move_group.stop();  // 中止当前 FollowJointTrajectory
      RCLCPP_INFO(node->get_logger(), "[cancel] stop() requested");

    } else if (k == 'r' || k == 'R') {
      // 重新规划（从当前状态）
      move_group.stop();
      move_group.clearPoseTargets();
      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(goal);
      auto ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      has_plan = ok;
      RCLCPP_INFO(node->get_logger(), "[replan] %s", ok ? "SUCCESS" : "FAILED");

    } else if (k == 'y') {
      // 局部Y轴 +10mm
      auto yaxis = local_y_axis_in_planning(goal.pose.orientation);
      goal.pose.position.x += 0.01 * yaxis.x;
      goal.pose.position.y += 0.01 * yaxis.y;
      goal.pose.position.z += 0.01 * yaxis.z;
      has_plan = false;
      RCLCPP_INFO(node->get_logger(),
                  "[offset +10mm local Y] new goal: (%.3f, %.3f, %.3f)",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

    } else if (k == 'Y') {
      // 局部Y轴 -10mm
      auto yaxis = local_y_axis_in_planning(goal.pose.orientation);
      goal.pose.position.x -= 0.01 * yaxis.x;
      goal.pose.position.y -= 0.01 * yaxis.y;
      goal.pose.position.z -= 0.01 * yaxis.z;
      has_plan = false;
      RCLCPP_INFO(node->get_logger(),
                  "[offset -10mm local Y] new goal: (%.3f, %.3f, %.3f)",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

    } else {
      RCLCPP_WARN(node->get_logger(), "Unknown key: %c", k);
    }
  }

  rclcpp::shutdown();
  spin_thr.join();
  return 0;
}
