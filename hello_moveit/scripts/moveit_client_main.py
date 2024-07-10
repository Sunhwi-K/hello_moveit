#!/usr/bin/env python3
"""
Sample script to run Moveit clients
"""
import copy
import threading

import rclpy
from geometry_msgs.msg import Pose
from hello_moveit.moveit_client import (apply_collision_object,
                                        apply_collision_object_from_mesh,
                                        attach_hand, check_collision,
                                        compute_fk, compute_ik, detach_hand,
                                        plan_execute_cartesian_path,
                                        plan_execute_poses)
from moveit_msgs.msg import CollisionObject


def main() -> None:
    """
    Sample sequence of Moveit clients
    """
    rclpy.init()
    node = rclpy.create_node('oreore')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    apply_collision_object(node)
    apply_collision_object_from_mesh(node)
    attach_hand(node)
    check_collision(node)

    # set target pose1
    msg = Pose()
    msg.orientation.w = -0.5
    msg.orientation.x = 0.5
    msg.orientation.y = 0.5
    msg.orientation.z = -0.5
    msg.position.x = -0.696
    msg.position.y = 0.052
    msg.position.z = 0.464
    node.get_logger().info(f'target tcp pose : {msg}')

    # get target joint state for moving from current pose to the target pose1
    target_joint_state = compute_ik(node, target_tcp_pose=msg)
    node.get_logger().info(f'target joint state : {target_joint_state}')

    # send target pose1
    plan_execute_poses(node, msg)

    # get current pose of tcp
    current_tcp_pose = compute_fk(node)
    node.get_logger().info(f'current tcp pose : {current_tcp_pose}')

    # send target pose2
    msg = Pose()
    msg.orientation.w = -0.5
    msg.orientation.x = 0.5
    msg.orientation.y = 0.5
    msg.orientation.z = -0.5
    msg.position.x = -0.696
    msg.position.y = 0.0519
    msg.position.z = 0.154
    plan_execute_poses(node, msg)

    # send catesian path
    waypoint1 = copy.deepcopy(msg)
    waypoint2 = copy.deepcopy(msg)
    waypoint1.position.y += 0.2
    plan_execute_cartesian_path(node, [waypoint1, waypoint2])

    # remove object
    apply_collision_object_from_mesh(node, CollisionObject.REMOVE)
    apply_collision_object(node, CollisionObject.REMOVE)
    detach_hand(node)

    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
