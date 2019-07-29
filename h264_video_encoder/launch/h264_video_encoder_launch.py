# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.
import launch
import launch_ros.actions

# Argument names
NODE_NAME = "node_name"

def generate_launch_description():
  ld = launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(
      NODE_NAME,
      default_value="h264_video_encoder",
    ),
   ])
  encoder_node = launch_ros.actions.Node(
      package="h264_video_encoder",
      node_executable="h264_video_encoder",
      node_name=launch.substitutions.LaunchConfiguration(NODE_NAME),
  )

  ld.add_action(encoder_node)
  
  return ld


if __name__ == "__main__":
  generate_launch_description()
