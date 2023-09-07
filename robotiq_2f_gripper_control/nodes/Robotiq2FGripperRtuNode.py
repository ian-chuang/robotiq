#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
ROS node for controling a Robotiq 2F gripper using the Modbus RTU protocol.

The script takes as an argument the IP address of the gripper. It initializes a baseRobotiq2FGripper object and adds a comModbusTcp client to it. It then loops forever, reading the gripper status and updating its command. The gripper status is published on the 'Robotiq2FGripperRobotInput' topic using the 'Robotiq2FGripper_robot_input' msg type. The node subscribes to the 'Robotiq2FGripperRobotOutput' topic for new commands using the 'Robotiq2FGripper_robot_output' msg type. Examples are provided to control the gripper (Robotiq2FGripperSimpleController.py) and interpreting its status (Robotiq2FGripperStatusListener.py).
"""

import sys
import os
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as inputMsg
from robotiq_modbus_rtu.comModbusRtu import ComModbusRtu, ReadGripperError, WriteGripperError
import robotiq_2f_gripper_control.baseRobotiq2FGripper
import rospy
import roslib
roslib.load_manifest('robotiq_2f_gripper_control')
roslib.load_manifest('robotiq_modbus_rtu')


def mainLoop():

    rospy.init_node('robotiq2FGripper')

    device = rospy.get_param('~port')
    control_topic = rospy.get_param('~control_topic', 'gripper/control')
    state_topic = rospy.get_param('~state_topic', 'gripper/state')

    # Gripper is a 2F with a TCP connection
    gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
    gripper.client = ComModbusRtu()

    # We connect to the address received as an argument
    success = gripper.client.connectToDevice(device)

    if not success:
        rospy.logerr("Failed to connect and read from gripper")
        raise ConnectionError("Unable to establish connection")
    
    def reset():
        cmd = outputMsg()
        cmd.rACT = 0
        gripper.refreshCommand(cmd)
        gripper.sendCommand()

    def activate(timeout=10):
        cmd = outputMsg()
        cmd.rACT = 1
        gripper.refreshCommand(cmd)
        gripper.sendCommand()
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if timeout >= 0. and rospy.get_time() - start_time > timeout:
                return False

            status = gripper.getStatus()
            is_ready = status.gSTA == 3 and status.gACT == 1
            if is_ready: return True

            r.sleep()

        return False
    
    reset()
    success = activate()

    if not success:    
        rospy.logerr("Failed to activate gripper")
        raise ConnectionError("Unable to activate gripper")

    # The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
    pub = rospy.Publisher(state_topic,
                          inputMsg, queue_size=1)

    # The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
    rospy.Subscriber(control_topic,
                     outputMsg, gripper.refreshCommand)

    # We loop
    while not rospy.is_shutdown():
        try:
            # Get and publish the Gripper status
            status = gripper.getStatus()
            pub.publish(status)

            # Send the most recent command
            gripper.sendCommand()

        except ReadGripperError as e:
            rospy.logerr(e)
        except WriteGripperError as e:
            rospy.logerr(e)
        


if __name__ == '__main__':
    mainLoop()

