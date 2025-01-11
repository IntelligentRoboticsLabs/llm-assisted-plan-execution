# Copyright 2024 IntelligentRoboticsLab
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

import base64
import io
import os
import uuid
import google.generativeai as genai
from mistralai import Mistral
from mistralai.models import UserMessage, AssistantMessage
from ollama import Options, Client
import openai
from google.generativeai.types import HarmBlockThreshold, HarmCategory
import rclpy
from plansys2_examples_msgs.action import QueryModel
from std_msgs.msg import String
from PIL import Image
from rclpy.action import ActionServer, GoalResponse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.lifecycle import (LifecycleNode, LifecycleState,
                             TransitionCallbackReturn)

import ros2_numpy as rnp


class QueryServer(LifecycleNode):
    """QueryServer is a lifecycle node for handling query to the models."""

    def __init__(self) -> None:
        """Initialize the QueryServer."""
        super().__init__('planning_node')

        self._api_key = None
        self._model_provider = None
        self._temperature = None
        self._max_tokens = None
        self._top_p = None
        self._model_name = None
        self._model_provider = None
        self._api_key = None
        self._answer_pub = None
        self._chat_sessions = {}

        # params
        # declare params without intial value to rise an error
        # when not specified
        self.declare_parameter('env_api_key', rclpy.Parameter.Type.STRING)
        self.declare_parameter('model_provider', rclpy.Parameter.Type.STRING)
        self.declare_parameter('model_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('temperature', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_tokens', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('top_p', rclpy.Parameter.Type.DOUBLE)

    def handle_goal(self, goal_request):
        """Handle the goal action."""
        if goal_request.chat_id == 0:
            chat_id = uuid.uuid4().int % 256
            if self._model_provider == 'openai':
                openai.api_key = os.getenv(self._api_key)
                self._chat_sessions[chat_id] = [openai.OpenAI(), []]
                goal_request.chat_id = chat_id
            elif self._model_provider == 'google':
                genai.configure(api_key=os.environ[self._api_key])
                model = genai.GenerativeModel(self._model_name)
                self._chat_sessions[chat_id] = [model.start_chat(), None]
                goal_request.chat_id = chat_id
            elif self._model_provider == 'ollama':
                self._chat_sessions[chat_id] = [Client(), []]
                goal_request.chat_id = chat_id
            elif self._model_provider == 'mistral':
                self._chat_sessions[chat_id] = [Mistral(api_key=os.environ[self._api_key]), []]
                goal_request.chat_id = chat_id
        elif goal_request.chat_id not in self._chat_sessions:
            self.get_logger().warning('Chat id does not exist, goal is rejected')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def execute_query(self, goal_handle) -> None:
        """Execute the query action."""
        self.get_logger().info('Execue query with chat id: {}'.format(goal_handle.request.chat_id))

        feedback_msg = QueryModel.Feedback()
        result_msg = QueryModel.Result()
        result_msg.chat_id = goal_handle.request.chat_id
        full_text = ''

        if goal_handle.request.query_text != '':

            if not goal_handle.request.images:

                if self._model_provider == 'openai':

                    new_message = [
                        {
                            'role': 'user',
                            'content': [
                                {
                                    'type': 'text',
                                    'text': goal_handle.request.query_text
                                }
                            ]
                        }
                    ]

                    self._chat_sessions[goal_handle.request.chat_id][1].extend(new_message)

                    stream = self._chat_sessions[goal_handle.request.chat_id][0].chat.completions.create(
                        model=self._model_name,
                        temperature=self._temperature,
                        max_tokens=self._max_tokens,
                        top_p=self._top_p,
                        messages=self._chat_sessions[goal_handle.request.chat_id][1],
                        stream=True,
                    )

                    for chunk in stream:
                        if chunk.choices[0].delta.content is not None:
                            full_text += chunk.choices[0].delta.content
                            feedback_msg.feedback_text = full_text
                            goal_handle.publish_feedback(feedback_msg)

                    result_msg.result_text = full_text
                    self._answer_pub.publish(String(data=full_text))
                    goal_handle.succeed()
                    return result_msg

                elif self._model_provider == 'google':

                    response = self._chat_sessions[goal_handle.request.chat_id][0].send_message(
                        goal_handle.request.query_text,
                        generation_config=genai.types.GenerationConfig(
                            # Only one candidate for now.
                            top_p=self._top_p,
                            max_output_tokens=self._max_tokens,
                            temperature=self._temperature
                        ),
                        safety_settings={
                            HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT:
                                HarmBlockThreshold.BLOCK_NONE
                        },  # Optional
                        stream=True)

                    for chunk in response:
                        full_text += chunk.text
                        feedback_msg.feedback_text = full_text
                        goal_handle.publish_feedback(feedback_msg)

                    result_msg.result_text = full_text
                    self._answer_pub.publish(String(data=full_text))
                    goal_handle.succeed()
                    return result_msg

                elif self._model_provider == 'ollama':
                    # new_message = [
                    #     {
                    #         'role': 'user',
                    #         'content': goal_handle.request.query_text
                    #     },
                    # ]

                    # self._chat_sessions[goal_handle.request.chat_id][1].extend(new_message)

                    generate_params = {
                        'model': self._model_name,
                        'options': Options(temperature=self._temperature,
                                           top_p=self._top_p,),
                        'messages': self._chat_sessions[goal_handle.request.chat_id][1]
                        + [
                            {
                                'role': 'user',
                                'content': goal_handle.request.query_text
                            }
                        ],
                        'stream': True
                    }
                    stream = self._chat_sessions[goal_handle.request.chat_id][0].chat(
                        **generate_params
                    )

                    for chunk in stream:
                        if chunk['message']['content'] is not None:
                            full_text += chunk['message']['content']
                            feedback_msg.feedback_text = full_text
                            goal_handle.publish_feedback(feedback_msg)
    
                    self._chat_sessions[goal_handle.request.chat_id][1].append({
                        'role': 'user',
                        'content': goal_handle.request.query_text
                    })
                    self._chat_sessions[goal_handle.request.chat_id][1].append({
                        'role': 'assistant',
                        'content': full_text
                    })
                    result_msg.result_text = full_text
                    self._answer_pub.publish(String(data=full_text))
                    goal_handle.succeed()
                    return result_msg

                elif self._model_provider == 'mistral':

                    new_message = UserMessage(content=goal_handle.request.query_text)
                    self._chat_sessions[goal_handle.request.chat_id][1].append(new_message)
                    response = self._chat_sessions[goal_handle.request.chat_id][0].chat.stream(
                        messages=self._chat_sessions[goal_handle.request.chat_id][1],
                        model=self._model_name,
                        top_p=self._top_p,
                        temperature=self._temperature,
                        max_tokens=self._max_tokens,                        
                    )
                    for chunk in response:
                        full_text += chunk.data.choices[0].delta.content
                        feedback_msg.feedback_text = full_text
                        goal_handle.publish_feedback(feedback_msg)

                    self._chat_sessions[goal_handle.request.chat_id][1].append(
                        AssistantMessage(content=full_text))

                    result_msg.result_text = full_text
                    self._answer_pub.publish(String(data=full_text))
                    goal_handle.succeed()
                    return result_msg
                else:
                    self.get_logger().error(
                        'Model provider not supported, '
                        'please use openai or google'
                    )
                    goal_handle.abort()

            else:
                self.get_logger().info('QueryServer: execute_query with images'
                                       )
                self.get_logger().debug('Query: {}'.format(
                    goal_handle.request.query_text))
                encoded_images = []
                images = []
                for image in goal_handle.request.images:

                    np_array = rnp.image.image_to_numpy(image)
                    with Image.fromarray(np_array) as img:

                        # todo(juandpenan) resize?
                        img_byte_arr = io.BytesIO()
                        img.show()
                        img.save(img_byte_arr, format='PNG', quality=85)

                        image_base64 = base64.b64encode(
                            img_byte_arr.getvalue()
                        ).decode('utf-8')
                        if self._model_provider == 'ollama':
                            encoded_images.append(image_base64)
                        elif self._model_provider == 'openai':
                            encoded_images.append(
                                f'data:image/jpeg;base64,{image_base64}'
                            )
                        images.append(img)

                if self._model_provider == 'openai':

                    new_message = [
                        {
                            'role': 'user',
                            'content': [
                                {
                                    'type': 'text',
                                    'text': goal_handle.request.query_text
                                },
                                *[
                                    {
                                        'type': 'image_url',
                                        'image_url': {
                                            'url': image_base64,
                                        },
                                    }
                                    for image_base64 in encoded_images
                                ]
                            ]
                        }
                    ]

                    self._chat_sessions[goal_handle.request.chat_id][1].extend(new_message)

                    stream = self._chat_sessions[goal_handle.request.chat_id][0].chat.completions.create(
                        model=self._model_name,
                        temperature=self._temperature,
                        max_tokens=self._max_tokens,
                        top_p=self._top_p,
                        messages=self._chat_sessions[goal_handle.request.chat_id][1],
                        stream=True,
                    )

                    for chunk in stream:
                        if chunk.choices[0].delta.content is not None:
                            full_text += chunk.choices[0].delta.content
                            feedback_msg.feedback_text = full_text
                            goal_handle.publish_feedback(feedback_msg)

                    result_msg.result_text = full_text
                    self._answer_pub.publish(String(data=full_text))
                    full_text = ''
                    goal_handle.succeed()
                    return result_msg

                elif self._model_provider == 'google':
                    self.get_logger().debug('Executing with google provider')
                    new_chat = images
                    new_chat.append(goal_handle.request.query_text)
                    response = self._chat_sessions[goal_handle.request.chat_id][0].send_message(
                        new_chat,
                        generation_config=genai.types.GenerationConfig(
                            top_p=self._top_p,
                            max_output_tokens=self._max_tokens,
                            temperature=self._temperature,
                        ),
                        stream=True)
                    self.get_logger().debug('LLM answered')

                    for chunk in response:
                        full_text += chunk.text
                        feedback_msg.feedback_text = full_text
                        goal_handle.publish_feedback(feedback_msg)

                    result_msg.result_text = full_text
                    self._answer_pub.publish(String(data=full_text))
                    goal_handle.succeed()
                    full_text = ''
                    return result_msg

                elif self._model_provider == 'ollama':
                    # new_message = [
                    #     {
                    #         'role': 'user',
                    #         'content': goal_handle.request.query_text,
                    #         'images': encoded_images
                    #     },
                    # ]
                   
                    # self._chat_sessions[goal_handle.request.chat_id][1].extend(new_message)
                    
                    generate_params = {
                        'model': self._model_name,
                        'options': Options(temperature=self._temperature,
                                           top_p=self._top_p,),
                        'messages': self._chat_sessions[goal_handle.request.chat_id][1]
                        + [
                            {
                                'role': 'user',
                                'content': goal_handle.request.query_text,
                                'images': encoded_images
                            }
                        ],
                        'stream': True  # Set to True if you want real-time responses
                    }

                    stream = self._chat_sessions[goal_handle.request.chat_id][0].chat(
                        **generate_params
                    )

                    for chunk in stream:
                        if chunk['message']['content'] is not None:
                            full_text += chunk['message']['content']
                            feedback_msg.feedback_text = full_text
                            goal_handle.publish_feedback(feedback_msg)

                    self._chat_sessions[goal_handle.request.chat_id][1].append({
                        'role': 'user',
                        'content': goal_handle.request.query_text,
                        'images': encoded_images
                    })
                    self._chat_sessions[goal_handle.request.chat_id][1].append({
                        'role': 'assistant',
                        'content': full_text
                    })
                    result_msg.result_text = full_text
                    self._answer_pub.publish(String(data=full_text))
                    goal_handle.succeed()
                    full_text = ''
                    return result_msg
                    
                else:
                    self.get_logger().error('Model provider not supported, '
                                            'please use openai or google')
                    goal_handle.abort()
        else:
            self.get_logger().warning('Please provide a query text')
            goal_handle.abort()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle the configuration transition."""
        self.get_logger().info('QueryServer: on_configure')

        self._model_provider = self.get_parameter(
            'model_provider').get_parameter_value().string_value
        self._api_key = self.get_parameter(
            'env_api_key').get_parameter_value().string_value
        self._model_name = self.get_parameter(
            'model_name').get_parameter_value().string_value
        self._top_p = self.get_parameter(
            'top_p').get_parameter_value().double_value
        self._temperature = self.get_parameter(
            'temperature').get_parameter_value().double_value
        self._max_tokens = self.get_parameter(
            'max_tokens').get_parameter_value().integer_value
        if (self._model_provider != 'openai' and 
                self._model_provider != 'google' and 
                self._model_provider != 'ollama' and
                self._model_provider != 'mistral'):
            self.get_logger().error('Model provider not supported, '
                                    'please use openai or google')
            return TransitionCallbackReturn(success=False)

        # ros comunications
        self._action_server = ActionServer(
            self,
            QueryModel,
            'query_model',
            execute_callback=self.execute_query,
            goal_callback=self.handle_goal,
            # handle_cancel=self.handle_cancel,
            # handle_accepted=self.execute_query
            )
        
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._answer_pub = self.create_publisher(String, 'last_answer', qos)

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle the activation transition."""
        self.get_logger().info('QueryServer: on_activate')

        return super().on_activate(state)


def main():
    """Initialize and run the QueryServer node."""
    rclpy.init()
    node = QueryServer()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
