#!/usr/bin/env python3
"""
Script for the apple_ik_collision_server package.

This script:
1. Fetches the current planning scene from MoveIt (/get_planning_scene).
2. Extracts the collision object poses and IDs (e.g. apples) from the planning scene.
3. Sends a detailed request (IkCollisionRequest) with the extracted object poses to the
   apple_ik_collision_checker node (/apple_ik_collision_check) to solve IK and check collisions.
"""

import argparse
import sys
import rclpy
from rclpy.node import Node

import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import apple_msgs.srv


class PlanningSceneIKCollisionClient(Node):
    def __init__(
        self,
        apple_ik_service_name: str = '/apple_ik_collision_check',
        planning_scene_service_name: str = '/get_planning_scene',
    ):
        super().__init__('planning_scene_ik_collision_client')
        self.apple_ik_service_name = apple_ik_service_name
        self.planning_scene_service_name = planning_scene_service_name

        self.get_logger().info(
            f'Initializing client with apple IK service: "{self.apple_ik_service_name}", '
            f'planning scene service: "{self.planning_scene_service_name}"'
        )

        self.ps_client = self.create_client(
            moveit_msgs.srv.GetPlanningScene,
            self.planning_scene_service_name,
        )

        self.ik_collision_client = self.create_client(
            apple_msgs.srv.IkCollisionRequest,
            self.apple_ik_service_name,
        )

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """Wait for required services to become available."""
        self.get_logger().info('Waiting for services...')

        if not self.ps_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'Planning scene service "{self.planning_scene_service_name}" is not available!'
            )
            return False

        if not self.ik_collision_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'Apple IK collision check service "{self.apple_ik_service_name}" is not available!'
            )
            return False

        self.get_logger().info('All required services are available.')
        return True

    def get_current_planning_scene(self) -> moveit_msgs.msg.PlanningScene:
        """Fetch the full current planning scene from MoveIt."""
        self.get_logger().info('Querying current planning scene from MoveIt...')
        req = moveit_msgs.srv.GetPlanningScene.Request()

        # Request full scene components (ROBOT_STATE, WORLD_OBJECT_GEOMETRY, OCTOMAP, etc.)
        req.components.components = (
            moveit_msgs.msg.PlanningSceneComponents.ROBOT_STATE
            | moveit_msgs.msg.PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | moveit_msgs.msg.PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | moveit_msgs.msg.PlanningSceneComponents.WORLD_OBJECT_NAMES
            | moveit_msgs.msg.PlanningSceneComponents.OCTOMAP
            | moveit_msgs.msg.PlanningSceneComponents.TRANSFORMS
            | moveit_msgs.msg.PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | moveit_msgs.msg.PlanningSceneComponents.LINK_PADDING_AND_SCALING
        )

        future = self.ps_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res is None:
            raise RuntimeError('Failed to get planning scene response from MoveIt.')

        return res.scene

    def extract_objects(
        self,
        planning_scene: moveit_msgs.msg.PlanningScene,
        prefix_filter: str = '',
        base_frame: str = 'world',
    ) -> tuple[geometry_msgs.msg.PoseArray, list[str], list[float]]:
        """Extract object poses, IDs, and default costs from the planning scene collision objects."""
        pose_array = geometry_msgs.msg.PoseArray()
        pose_array.header.frame_id = base_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        object_ids = []
        costs = []

        collision_objects = planning_scene.world.collision_objects
        self.get_logger().info(
            f'Found {len(collision_objects)} total collision object(s) in planning scene.'
        )

        for obj in collision_objects:
            # Filter by prefix if specified (e.g. "apple" or "apple_")
            if prefix_filter and not obj.id.startswith(prefix_filter):
                continue

            pose = None
            # 1. Check primitive_poses (e.g. sphere or box primitive)
            if len(obj.primitive_poses) > 0:
                pose = obj.primitive_poses[0]
            # 2. Check mesh_poses
            elif len(obj.mesh_poses) > 0:
                pose = obj.mesh_poses[0]
            # 3. Check direct object pose
            elif (
                obj.pose.orientation.w != 0.0
                or obj.pose.position.x != 0.0
                or obj.pose.position.y != 0.0
                or obj.pose.position.z != 0.0
            ):
                pose = obj.pose

            if pose is not None:
                # If frame is explicitly in obj header, note it
                if obj.header.frame_id:
                    pose_array.header.frame_id = obj.header.frame_id

                pose_array.poses.append(pose)
                object_ids.append(obj.id)
                # Default cost per object (1.0)
                costs.append(1.0)

        return pose_array, object_ids, costs

    def send_ik_collision_request(
        self,
        apple_poses: geometry_msgs.msg.PoseArray,
        apple_ids: list[str],
        apple_costs: list[float],
        is_scene_new: bool = True,
    ) -> bool:
        """Send IkCollisionRequest to the apple_ik_collision_checker node."""
        self.get_logger().info(
            f'Sending IkCollisionRequest with {len(apple_poses.poses)} object poses '
            f'(is_scene_new={is_scene_new}) to "{self.apple_ik_service_name}"...'
        )

        req = apple_msgs.srv.IkCollisionRequest.Request()
        req.is_scene_new = is_scene_new
        req.apple_poses = apple_poses
        req.apple_ids = apple_ids
        req.apple_costs = apple_costs

        future = self.ik_collision_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res is None:
            raise RuntimeError('IkCollisionRequest service call returned None.')

        return res.new_configurations_added


def parse_args():
    parser = argparse.ArgumentParser(
        description='Extract object poses from MoveIt planning scene and send to apple_ik_collision node'
    )
    parser.add_argument(
        '--service',
        '-s',
        type=str,
        default='/apple_ik_collision_check',
        help='Name of the apple IK collision service (default: /apple_ik_collision_check)',
    )
    parser.add_argument(
        '--planning-scene-service',
        '-p',
        type=str,
        default='/get_planning_scene',
        help='Name of the GetPlanningScene service (default: /get_planning_scene)',
    )
    parser.add_argument(
        '--filter',
        '-f',
        type=str,
        default='',
        help='Prefix filter for object IDs (e.g. "apple" or "apple_"). Default is empty (matches all).',
    )
    parser.add_argument(
        '--base-frame',
        type=str,
        default='world',
        help='Base reference frame for the pose array (default: world).',
    )
    parser.add_argument(
        '--append',
        action='store_true',
        help='Set is_scene_new=False to append/keep existing solutions instead of overwriting.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    client = PlanningSceneIKCollisionClient(
        apple_ik_service_name=args.service,
        planning_scene_service_name=args.planning_scene_service,
    )

    try:
        if not client.wait_for_services(timeout_sec=10.0):
            sys.exit(1)

        # 1. Fetch current planning scene
        planning_scene = client.get_current_planning_scene()

        # 2. Extract object poses and IDs
        prefix = args.filter
        poses, ids, costs = client.extract_objects(
            planning_scene=planning_scene,
            prefix_filter=prefix,
            base_frame=args.base_frame,
        )

        if len(poses.poses) == 0:
            client.get_logger().warn(
                f'No collision objects found matching filter "{prefix}". '
                f'Make sure objects are loaded into the MoveIt planning scene.'
            )
            sys.exit(1)

        print('\n' + '=' * 70)
        print(f' EXTRACTED {len(poses.poses)} OBJECT(S) FROM PLANNING SCENE')
        print('=' * 70)
        for i, (obj_id, pose) in enumerate(zip(ids, poses.poses)):
            pos = pose.position
            ori = pose.orientation
            print(
                f'[{i + 1:3d}] ID: {obj_id:<20} '
                f'Pos: ({pos.x:+.3f}, {pos.y:+.3f}, {pos.z:+.3f}) '
                f'Ori: ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})'
            )
        print('=' * 70 + '\n')

        # 3. Send request to apple_ik_collision node
        is_scene_new = not args.append
        client.get_logger().info('Sending request to solve IK and check collisions...')
        new_configs_added = client.send_ik_collision_request(
            apple_poses=poses,
            apple_ids=ids,
            apple_costs=costs,
            is_scene_new=is_scene_new,
        )

        # 4. Report result
        print('=' * 70)
        if new_configs_added:
            print('\033[92mSUCCESS: New collision-free configurations found and saved to CSV!\033[0m')
        else:
            print('\033[91mWARNING: No new valid collision-free configurations were added.\033[0m')
        print('=' * 70 + '\n')

    except KeyboardInterrupt:
        client.get_logger().info('Interrupted by user.')
    except Exception as e:
        client.get_logger().error(f'Execution error: {e}')
        raise e
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
