# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import unittest

import launch
import launch_testing
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from plansys2_examples_msgs.action import QueryModel
from PIL import Image
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import qos_profile_action_status_default
from sensor_msgs.msg import Image as ImageMsg

import ros2_numpy as rnp

# TODO(jacobperron) Reduce fudge once wait_for_service uses node graph events
TIME_FUDGE = 0.3

# TODO(juandpenan) Update to use python interpreter from venv

# python_interpreter = os.path.join(get_package_share_directory('planning_system'),
#                                   '../../../../../../parce/bin/pyhton')
# script_path = os.path.realpath(__file__)

# with open(script_path, 'r') as file:
#     lines = file.readlines()
# lines[0] = f'#!{python_interpreter}\n'

# with open(script_path, 'w') as file:
#     file.writelines(lines)


# @pytest.mark.launch_test
# def generate_test_description():

#     config = os.path.join(
#         get_package_share_directory('planning_system'),
#         'config/planning_server.yaml'
#     )

#     # activate a python venve :
#     activate_venv = ExecuteProcess(
#         cmd=['bash', '-c', 
#             'source ' + os.path.join(get_package_share_directory('planning_system'), '../../../../../../parce/bin/activate')],
#         name='activate_venv'
#     )
 
#     server = Node(
#         package='planning_system',
#         executable='planning_node',
#         parameters=[config],
#     )

#     context = {}

#     return (
#         LaunchDescription(
#             [
#                 # activate_venv,
#                 server,
#                 # Start test after 1s - gives time for the map_loader to finish initialization
#                 launch.actions.TimerAction(
#                     period=2.0, actions=[launch_testing.actions.ReadyToTest()]
#                 ),
#             ]
#         ),
#         context,
#     )


class TestActionClient(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        img_1 = Image.open(get_package_share_directory('planning_system') + '/test/image_1.png')
        img_2 = Image.open(get_package_share_directory('planning_system') + '/test/image_2.jpg')

        cls.img_msg_1 = rnp.msgify(ImageMsg, np.array(img_1), encoding='rgb8')
        cls.img_msg_2 = rnp.msgify(ImageMsg, np.array(img_2), encoding='rgb8')

        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.node = rclpy.create_node('TestActionClient', context=cls.context)
        # cls.server = QueryModelServer()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def setUp(self):
        self.feedback = None

    def feedback_callback(self, feedback):
        self.feedback = feedback

    def result_callback(self, future):
        self.node.get_logger().info('Got result')
        self.result = future.result().result
        self.node.get_logger().info('Result : {0}'.format(self.result.result_text))

    def timed_spin(self, duration):
        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)

    def test_only_text(self):
        ac = ActionClient(self.node, QueryModel, 'query_model')
        try:
            goal = QueryModel.Goal()
            goal.query_text = 'How are you today?'
            self.assertTrue(ac.wait_for_server(timeout_sec=4.0))
            self.node.get_logger().info('Server is alive')
            self.node.get_logger().info('Sending goal')
            future = ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future, self.executor)
            self.node.get_logger().info('Goal sent')
            self.assertTrue(future.done())
            self.node.get_logger().info('Future is done [only_text_test] : {0}'.format(future.done()))
            goal_handle = future.result()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
            rclpy.spin_until_future_complete(self.node,
                                             result_future,
                                             self.executor)
            self.assertTrue(goal_handle.accepted)
            self.node.get_logger().info('goal accepted : {0}'.format(goal_handle.accepted))
        finally:
            ac.destroy()

    def test_with_one_image(self):
        ac = ActionClient(self.node, QueryModel, 'query_model')
        try:
            goal = QueryModel.Goal()
            goal.query_text = 'What is the color of the animal?'
            self.node.get_logger().info('image type: {0}'.format(type(self.img_msg_1)))
            goal.images = [self.img_msg_1]
            self.assertTrue(ac.wait_for_server(timeout_sec=4.0))
            self.node.get_logger().info('Server is alive')
            self.node.get_logger().info('Sending goal')
            future = ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future, self.executor)
            self.node.get_logger().info('Goal sent')
            self.assertTrue(future.done())
            self.node.get_logger().info('Future is done')
            goal_handle = future.result()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
            rclpy.spin_until_future_complete(self.node, result_future, self.executor)
            self.assertTrue(goal_handle.accepted)
        finally:
            ac.destroy()

    def test_with_one_multile_images(self):
        ac = ActionClient(self.node, QueryModel, 'query_model')
        try:
            goal = QueryModel.Goal()
            goal.query_text = 'Describe the animals in the pictures'
            goal.images = [self.img_msg_1, self.img_msg_2]
            self.assertTrue(ac.wait_for_server(timeout_sec=4.0))
            self.node.get_logger().info('Server is alive')
            self.node.get_logger().info('Sending goal')
            future = ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future, self.executor)
            self.node.get_logger().info('Goal sent')
            self.assertTrue(future.done())
            self.node.get_logger().info('Future is done')
            goal_handle = future.result()
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)
            rclpy.spin_until_future_complete(self.node, result_future, self.executor)
            self.assertTrue(goal_handle.accepted)
        finally:
            ac.destroy()
    


if __name__ == '__main__':
    unittest.main()