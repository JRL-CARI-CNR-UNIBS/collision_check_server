#!/usr/bin/env python3
"""
Simple script to send a detailed collision check request with the current MoveIt
planning scene to the collision checker node (/collision_check).
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

import moveit_msgs.msg
import moveit_msgs.srv
import ik_solver_msgs.msg
import ik_solver_msgs.srv
import std_msgs.msg


# Default UR10e joint names used in apple picking cell
DEFAULT_JOINT_NAMES = [
    'ur10e_shoulder_pan_joint',
    'ur10e_shoulder_lift_joint',
    'ur10e_elbow_joint',
    'ur10e_wrist_1_joint',
    'ur10e_wrist_2_joint',
    'ur10e_wrist_3_joint',
]


class CollisionCheckClient(Node):
    def __init__(
        self,
        collision_service_name: str = '/collision_check',
        planning_scene_service_name: str = '/get_planning_scene',
    ):
        super().__init__('collision_check_client')
        self.collision_service_name = collision_service_name
        self.planning_scene_service_name = planning_scene_service_name

        self.get_logger().info(
            f'Initializing client with collision service: {self.collision_service_name}, '
            f'planning scene service: {self.planning_scene_service_name}'
        )

        self.ps_client = self.create_client(
            moveit_msgs.srv.GetPlanningScene,
            self.planning_scene_service_name,
        )

        self.cc_client = self.create_client(
            ik_solver_msgs.srv.CollisionChecking,
            self.collision_service_name,
        )

        self.display_pub = self.create_publisher(
            moveit_msgs.msg.DisplayRobotState,
            '/display_robot_state',
            10,
        )

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """Wait for required services to become available."""
        self.get_logger().info('Waiting for services to become available...')

        if not self.ps_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'Planning scene service "{self.planning_scene_service_name}" is not available!'
            )
            return False

        if not self.cc_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'Collision checking service "{self.collision_service_name}" is not available!'
            )
            return False

        self.get_logger().info('All required services are available.')
        return True

    def get_current_planning_scene(self) -> moveit_msgs.msg.PlanningScene:
        """Fetch the full current planning scene from MoveIt."""
        self.get_logger().info('Fetching current planning scene from MoveIt...')
        req = moveit_msgs.srv.GetPlanningScene.Request()

        # Request all planning scene components (full scene)
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
            raise RuntimeError('Failed to get planning scene response.')

        scene = res.scene
        self.get_logger().info(
            f'Planning scene received successfully. '
            f'World objects count: {len(scene.world.collision_objects)}, '
            f'Robot state joints count: {len(scene.robot_state.joint_state.name)}'
        )
        return scene

    def send_collision_request(
        self,
        configurations: list[list[float]],
        joint_names: list[str],
        planning_scene: moveit_msgs.msg.PlanningScene,
        detailed: bool = True,
    ) -> list[ik_solver_msgs.msg.CollisionResult]:
        """Send detailed collision check request to the node."""
        self.get_logger().info(
            f'Sending collision checking request for {len(configurations)} configuration(s) '
            f'(detailed={detailed})...'
        )

        req = ik_solver_msgs.srv.CollisionChecking.Request()
        req.detailed = detailed
        req.planning_scene = planning_scene
        req.joint_names = joint_names

        for cfg in configurations:
            sol = ik_solver_msgs.msg.Configuration()
            sol.configuration = [float(val) for val in cfg]
            req.solutions.append(sol)

        future = self.cc_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res is None:
            raise RuntimeError('Collision checking service call returned None.')

        return res.result

    def print_results(
        self,
        results: list[ik_solver_msgs.msg.CollisionResult],
        joint_names: list[str],
    ):
        """Print collision results in a detailed, structured format."""
        print('\n' + '=' * 70)
        print(' COLLISION CHECKING RESULTS (DETAILED)')
        print('=' * 70)

        num_total = len(results)
        num_free = 0
        num_collision = 0
        num_oob = 0

        for idx, res in enumerate(results):
            is_oob = res.out_of_bound
            is_collision = (res.feasibility != 1.0) and not is_oob
            is_free = (res.feasibility == 1.0) and not is_oob

            if is_free:
                num_free += 1
                status_str = '\033[92mCOLLISION-FREE\033[0m'
            elif is_collision:
                num_collision += 1
                status_str = '\033[91mIN COLLISION\033[0m'
            else:
                num_oob += 1
                status_str = '\033[93mOUT OF BOUNDS\033[0m'

            print(f'\nConfiguration #{idx + 1}: [{status_str}]')
            cfg_str = ', '.join([f'{j}={v:.3f}' for j, v in zip(joint_names, res.solution.configuration)])
            print(f'  Joint values: [{cfg_str}]')
            print(f'  Feasibility : {res.feasibility:.2f} (1.0 = feasible, 0.0 = collision/oob)')
            print(f'  Distance    : {res.distance:.4f} m')

            if is_oob:
                print(f'  Out-of-bound links: {list(res.out_of_bound_links)}')

            if is_collision or len(res.colliding_obj_first) > 0:
                print('  Colliding contacts:')
                for c1, c2 in zip(res.colliding_obj_first, res.colliding_obj_second):
                    print(f'    - {c1} <---> {c2}')

        print('\n' + '-' * 70)
        print(f'SUMMARY: Total: {num_total} | Free: {num_free} | Collision: {num_collision} | Out-of-Bounds: {num_oob}')
        print('=' * 70 + '\n')

    def publish_visualization(
        self,
        results: list[ik_solver_msgs.msg.CollisionResult],
        joint_names: list[str],
        base_frame: str = 'world',
        period_sec: float = 1.0,
    ):
        """Publish configurations to /display_robot_state with red highlight for colliding links."""
        self.get_logger().info(
            f'Publishing {len(results)} configuration(s) to /display_robot_state '
            f'every {period_sec}s...'
        )

        red_color = std_msgs.msg.ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        for idx, res in enumerate(results):
            msg = moveit_msgs.msg.DisplayRobotState()
            msg.state.is_diff = False
            msg.state.joint_state.header.stamp = self.get_clock().now().to_msg()
            msg.state.joint_state.header.frame_id = base_frame
            msg.state.joint_state.name = joint_names
            msg.state.joint_state.position = list(res.solution.configuration)
            msg.state.joint_state.velocity = [0.0] * len(joint_names)
            msg.state.joint_state.effort = [0.0] * len(joint_names)

            # Highlight colliding links in red
            colliding_links = set(res.colliding_obj_first + res.colliding_obj_second)
            for link_id in colliding_links:
                obj_color = moveit_msgs.msg.ObjectColor()
                obj_color.id = link_id
                obj_color.color = red_color
                msg.highlight_links.append(obj_color)

            self.display_pub.publish(msg)
            self.get_logger().info(
                f'Published configuration {idx + 1}/{len(results)} '
                f'(Colliding links: {len(colliding_links)})'
            )
            time.sleep(period_sec)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send detailed collision check request with current planning scene to collision check node'
    )
    parser.add_argument(
        '--service',
        '-s',
        type=str,
        default='/collision_check',
        help='Name of the collision check service (default: /collision_check)',
    )
    parser.add_argument(
        '--planning-scene-service',
        '-p',
        type=str,
        default='/get_planning_scene',
        help='Name of the GetPlanningScene service (default: /get_planning_scene)',
    )
    parser.add_argument(
        '--joint-names',
        '-j',
        nargs='+',
        default=DEFAULT_JOINT_NAMES,
        help='List of joint names (default: UR10e arm joint names)',
    )
    parser.add_argument(
        '--configs',
        '-c',
        nargs='+',
        action='append',
        type=float,
        help='Configuration joint values (e.g. -c 0.0 -1.57 1.57 0.0 0.0 0.0). Can specify multiple times.',
    )
    parser.add_argument(
        '--detailed',
        action='store_true',
        default=True,
        help='Request detailed collision info (distances, colliding contacts). Default is True.',
    )
    parser.add_argument(
        '--visualize',
        '-v',
        action='store_true',
        help='Publish results to /display_robot_state for RViz visualization.',
    )
    parser.add_argument(
        '--base-frame',
        type=str,
        default='world',
        help='Base frame id for visualization (default: world).',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    client = CollisionCheckClient(
        collision_service_name=args.service,
        planning_scene_service_name=args.planning_scene_service,
    )

    try:
        if not client.wait_for_services(timeout_sec=10.0):
            sys.exit(1)

        # 1. Fetch the latest current planning scene
        planning_scene = client.get_current_planning_scene()

        # 2. Determine configurations to check
        joint_names = args.joint_names

        if args.configs:
            configurations = args.configs
        else:
            # Default test configurations:
            # 1) Current robot state from planning scene (if available for these joints)
            # 2) Home / zero configuration
            # 3) A common arm picking posture
            configurations = []

            # Try extracting current values from planning scene
            if planning_scene.robot_state.joint_state.name:
                js = planning_scene.robot_state.joint_state
                js_map = dict(zip(js.name, js.position))
                if all(jn in js_map for jn in joint_names):
                    current_conf = [js_map[jn] for jn in joint_names]
                    configurations.append(current_conf)

            # Sample test configurations
            configurations.append([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
            configurations.append([0.5, -1.2, 1.3, -1.6, -1.5, 0.2])
            configurations.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        client.get_logger().info(f'Joint names: {joint_names}')
        client.get_logger().info(f'Checking {len(configurations)} configuration(s)...')

        # 3. Send detailed request with planning scene
        results = client.send_collision_request(
            configurations=configurations,
            joint_names=joint_names,
            planning_scene=planning_scene,
            detailed=args.detailed,
        )

        # 4. Print detailed results
        client.print_results(results, joint_names)

        # 5. Optional visualization
        if args.visualize:
            client.publish_visualization(
                results=results,
                joint_names=joint_names,
                base_frame=args.base_frame,
            )

    except KeyboardInterrupt:
        client.get_logger().info('Process interrupted by user.')
    except Exception as e:
        client.get_logger().error(f'Error occurred: {e}')
        raise e
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

