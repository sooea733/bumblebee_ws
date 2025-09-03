#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  // ROS 2 초기화
  rclcpp::init(argc, argv);

  // 노드 생성
  auto node = std::make_shared<rclcpp::Node>("arm_moveit_node");

  // 로그 출력
  RCLCPP_INFO(node->get_logger(), "Starting Arm MoveIt Node");

  // 스레드 실행 (비동기 콜백 처리)
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  auto spin_thread = std::thread([executor]() { executor->spin(); });

  // MoveIt 초기화 대기
  rclcpp::sleep_for(5s);

  // MoveGroupInterface 생성
  static const std::string PLANNING_GROUP = "arm";  // SRDF에서 그룹 이름 확인
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // MoveIt 정보 출력
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // 속도 및 가속도 설정
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);

  // 현재 포즈 정보 출력
  auto current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Current pose: Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)",
    current_pose.position.x, current_pose.position.y, current_pose.position.z,
    current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

  // 사용 가능한 named targets 출력
  std::vector<std::string> named_targets = move_group.getNamedTargets();
  if (!named_targets.empty()) {
    RCLCPP_INFO(node->get_logger(), "Available named targets:");
    for (const auto& target : named_targets) {
      RCLCPP_INFO(node->get_logger(), "  %s", target.c_str());
    }
  }

  // 포즈 타겟 설정
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;  // 회전 없음
  target_pose.position.x = 0.5;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.6;
  
  move_group.setPoseTarget(target_pose);
  
  // 현재 상태를 시작 상태로 설정
  move_group.setStartStateToCurrentState();

  // 플래닝 실행
  RCLCPP_INFO(node->get_logger(), "Planning to pose target...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));

  // 계획 성공 여부 확인 후 실행
  if (success) {
    if (plan.trajectory_.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(node->get_logger(), "Planned trajectory is empty! Execution aborted.");
    } else {
      RCLCPP_INFO(node->get_logger(), "Planning successful! Executing...");
      move_group.asyncExecute(plan);
      RCLCPP_INFO(node->get_logger(), "Execution complete!");
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  // 스핀 스레드 종료
  executor->cancel();
  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}
