#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto right_leg = moveit::planning_interface::MoveGroupInterface(node, "right_leg");
    right_leg.setMaxVelocityScalingFactor(1.0);
    right_leg.setMaxAccelerationScalingFactor(1.0);

    auto left_leg = moveit::planning_interface::MoveGroupInterface(node, "left_leg");

    // Named goal

    right_leg.setStartStateToCurrentState();
    right_leg.setNamedTarget("steady_right");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (right_leg.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1) {
        right_leg.execute(plan1);
    }

    right_leg.setStartStateToCurrentState();
    right_leg.setNamedTarget("home");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (right_leg.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2) {
        right_leg.execute(plan2);
    }

    // --------------------------------------------------------------------------------


    rclcpp::shutdown();
    spinner.join();
    return 0;
}