# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
#
# --------------------------------------------------------
# Based on:
# https://github.com/felixduvallet/allegro-hand-ros/blob/master/allegro_hand/src/allegro_hand/liballegro.py
# --------------------------------------------------------

import traceback
from typing import List, Tuple, Union

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.parameter_service import GetParameters, SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class AllegroRobot(Node):
    def __init__(
        self, hand_topic_prefix: str = "allegroHand", num_joints: int = 16
    ) -> None:
        """Simple python interface to the Allegro Hand.

        The AllegroClient is a simple python interface to an allegro
        robot hand.  It enables you to command the hand directly through
        python library calls (joint positions, joint torques, or 'named'
        grasps).

        The constructors sets up publishers and subscribes to the joint states
        topic for the hand.

        Note on hand topic names: The default topic (allegroHand/foo) can be
        remapped to a different topic prefix (allegroHand_0/foo) in one of two
        ways:
          1. pass in allegroHand_0 as the hand_topic_prefix
          2. remap *each* topic on the command line
             (allegroHand/joint_cmd:=allegroHand_0/joint_cmd)
        The first method is probably easier.

        :param hand_topic_prefix: The prefix to use for *all* hand
        topics (publishing & subscribing).

        :param num_joints: Number of expected joints, used when
        commanding joint positions.

        """
        super().__init__("hand_commander")

        # Topics (that can be remapped) for named graps
        # (ready/envelop/grasp/etc.), joint commands (position and
        # velocity), joint state (subscribing), and envelop torque. Note that
        # we can change the hand topic prefix (for example, to allegroHand_0)
        # instead of remapping it at the command line.
        hand_topic_prefix = hand_topic_prefix.rstrip("/")
        topic_grasp_command = "{}/lib_cmd".format(hand_topic_prefix)
        topic_joint_command = "{}/joint_cmd".format(hand_topic_prefix)
        topic_joint_state = "{}/joint_states".format(hand_topic_prefix)

        # Publishers for above topics.
        self.pub_grasp = self.create_publisher(String, topic_grasp_command, 10)
        self.pub_joint = self.create_publisher(JointState, topic_joint_command, 10)
        self.create_subscription(
            JointState, topic_joint_state, self._joint_state_callback, 10
        )
        self._joint_state = None

        self._num_joints = num_joints

        self.get_logger().info(
            "Allegro Client start with hand topic: {}".format(hand_topic_prefix)
        )

        # "Named" grasps are those provided by the bhand library. These can be
        # commanded directly and the hand will execute them. The keys are more
        # human-friendly names, the values are the expected names from the
        # allegro controller side. Multiple strings mapping to the same value
        # are allowed.
        self._named_grasps_mappings = {
            "home": "home",
            "off": "off",
            # "gravity_compensation": "gravcomp",
            # "gravity compensation": "gravcomp",
            # "gravity": "gravcomp",
            # "ready": "ready",
            # "three_finger_grasp": "grasp_3",
            # "three finger grasp": "grasp_3",
            # "four_finger_grasp": "grasp_4",
            # "four finger grasp": "grasp_4",
            # "index_pinch": "pinch_it",
            # "index pinch": "pinch_it",
            # "middle_pinch": "pinch_mt",
            # "middle pinch": "pinch_mt",
            # "envelop": "envelop",
        }
        self.joint_names = [f"allegro_joint_{i}.0" for i in range(self._num_joints)]

    def disconnect(self) -> None:
        """
        Disconnect the allegro client from the hand by sending the 'off'
        command. This is principally a convenience binding.

        Note that we don't actually 'disconnect', so you could technically
        continue sending other commands after this.
        """
        self.command_hand_configuration("off")

    def _joint_state_callback(self, data: JointState) -> None:
        self._joint_state = data

    def get_p_gain(self) -> List[float]:
        client = self.create_client(
            GetParameters, "/allegroHand/allegro_controller/get_parameters"
        )
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError("Wait for service timed out")

        request = GetParameters.Request()
        request.names = [f"gains_pd/{i}/p" for i in range(self.num_joints)]
        result = client.call(request)
        return [v.double_value for v in result.values]

    def set_p_gain(self, gain: List[float]) -> None:
        client = self.create_client(
            SetParameters, "/allegroHand/allegro_controller/set_parameters"
        )
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError("Wait for service timed out")

        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=f"gains_pd/{i}/p",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=gain[i]
                ),
            )
            for i in range(self.num_joints)
        ]
        result = client.call(request)
        assert all(
            [result.results[i].successful for i in range(self.num_joints)]
        ), "gain setting failed"

    def command_joint_position(self, desired_pose: List) -> bool:
        """
        Command a specific desired hand pose.

        The desired pose must be the correct dimensionality (self._num_joints).
        Only the pose is commanded, and **no bound-checking happens here**:
        any commanded pose must be valid or Bad Things May Happen. (Generally,
        values between 0.0 and 1.5 are fine, but use this at your own risk.)

        :param desired_pose: The desired joint configurations.
        :return: True if pose is published, False otherwise.
        """

        # Check that the desired pose can have len() applied to it, and that
        # the number of dimensions is the same as the number of hand joints.
        if (
            not hasattr(desired_pose, "__len__")
            or len(desired_pose) != self._num_joints
        ):
            self.get_logger().warn(
                "Desired pose must be a {}-d array: got {}.".format(
                    self._num_joints, desired_pose
                )
            )
            return False

        msg = JointState()  # Create and publish
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        try:
            msg.position.fromlist(list(desired_pose))
            self.pub_joint.publish(msg)
            self.get_logger().debug("Published desired pose.")
            return True
        except Exception:
            self.get_logger().error(traceback.format_exc())
            self.get_logger.warn(
                "Incorrect type for desired pose: {}.".format(desired_pose)
            )
            return False

    def command_joint_torques(self, desired_torques: List) -> bool:
        """
        Command a desired torque for each joint.

        The desired torque must be the correct dimensionality
        (self._num_joints). Similarly to poses, we do not sanity-check
        the inputs. As a rule of thumb, values between +- 0.5 are fine.

        :param desired_torques: The desired joint torques.
        :return: True if message is published, False otherwise.
        """

        # Check that the desired torque vector can have len() applied to it,
        # and that the number of dimensions is the same as the number of
        # joints. This prevents passing singletons or incorrectly-shaped lists
        # to the message creation (which does no checking).
        if (
            not hasattr(desired_torques, "__len__")
            or len(desired_torques) != self._num_joints
        ):
            self.get_logger().warn(
                "Desired torques must be a {}-d array: got {}.".format(
                    self._num_joints, desired_torques
                )
            )
            return False

        msg = JointState()  # Create and publish
        try:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.effort = desired_torques
            self.pub_joint.publish(msg)
            self.get_logger().debug("Published desired torques.")
            return True
        except Exception:
            self.get_logger().error(traceback.format_exc())
            self.get_logger().logwarn(
                "Incorrect type for desired torques: {}.".format(desired_torques)
            )
            return False

    def poll_joint_position(self, wait: bool = False) -> Union[Tuple, None]:
        """Get the current joint positions of the hand.

        :param wait: If true, waits for a 'fresh' state reading.
        :return: Joint positions, or None if none have been received.
        """
        if wait:  # Clear joint state and wait for the next reading.
            self._joint_state = None
            rate = self.create_rate(1000)
            while not self._joint_state:
                rate.sleep()
            self.destroy_rate(rate)  # type: ignore[unreachable]

        if self._joint_state:
            return (  # type: ignore[unreachable]
                self._joint_state.position,
                self._joint_state.effort,
            )
        else:
            return None

    def command_hand_configuration(self, hand_config: str) -> bool:
        """
        Command a named hand configuration (e.g., pinch_index, envelop,
        gravity_compensation).

        The internal hand configuration names are defined in the
        AllegroNodeGrasp controller file. More human-friendly names are used
        by defining them as 'shortcuts' in the _named_grasps_mapping variable.
        Multiple strings can map to the same commanded configuration.

        :param hand_config: A human-friendly string of the desired
        configuration.
        :return: True if the grasp was known and commanded, false otherwise.
        """

        # Only use known named grasps.
        if hand_config in self._named_grasps_mappings:
            # Look up conversion of string -> msg
            msg = String()
            msg.data = self._named_grasps_mappings[hand_config]
            self.get_logger().debug("Commanding grasp: {}".format(msg.data))
            self.pub_grasp.publish(msg)
            return True
        else:
            self.get_logger().warn(
                "Unable to command unknown grasp {}".format(hand_config)
            )
            return False

    def list_hand_configurations(self) -> List[str]:
        """
        :return: List of valid strings for named hand configurations (including
        duplicates).
        """
        return [k for k in self._named_grasps_mappings]

    @property
    def num_joints(self) -> int:
        return self._num_joints
