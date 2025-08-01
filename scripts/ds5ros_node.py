#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import traceback
from sensor_msgs.msg import Joy, JoyFeedbackArray
from pydualsense import *


class Ds5Ros(Node):
    def __init__(self):
        super().__init__('ds5ros_node')

        self.logging_prefix = "DS5_ROS: "
        self.node_state = 0
        self.prev_msg = Joy()
        self.dualsense = pydualsense()

        # Declare parameters with defaults
        self.declare_parameter("noderate", 50.0)
        self.declare_parameter("joy_sub", "joy/set_feedback")
        self.declare_parameter("joy_pub", "joy")
        self.declare_parameter("deadzone", 0.05)

        self.noderate = self.get_parameter("noderate").value
        self.joy_sub_topic = self.get_parameter("joy_sub").value
        self.joy_pub_topic = self.get_parameter("joy_pub").value
        self.deadzone = self.get_parameter("deadzone").value

        self.joy_pub = self.create_publisher(Joy, self.joy_pub_topic, 10)
        self.joy_sub = self.create_subscription(JoyFeedbackArray, self.joy_sub_topic, self.set_feedback, 10)

        self.maskR = 0xFF0000
        self.maskG = 0x00FF00
        self.maskB = 0x0000FF

        # Create main timer
        self.timer = self.create_timer(1.0 / self.noderate, self.main_loop)

    def set_feedback(self, msg):
        for feedback in msg.array:
            if feedback.type == 0:
                int_intensity = int(feedback.intensity)
                light_red = (int_intensity & self.maskR) >> 16
                light_green = (int_intensity & self.maskG) >> 8
                light_blue = (int_intensity & self.maskB)
                self.dualsense.light.setColorI(light_red, light_green, light_blue)
                continue

            if feedback.intensity > 1.0 or feedback.intensity < 0.0:
                self.get_logger().error(self.logging_prefix + 'intensity must be in range 0.0 - 1.0')
                continue

            feedback.intensity = feedback.intensity * 255.0

            if feedback.type == 1 and feedback.id == 2:
                self.dualsense.triggerL.setMode(TriggerModes.Rigid)
                self.dualsense.triggerL.setForce(1, int(feedback.intensity))
            elif feedback.type == 1 and feedback.id == 3:
                self.dualsense.triggerR.setMode(TriggerModes.Rigid)
                self.dualsense.triggerR.setForce(1, int(feedback.intensity))

    def joy_publish(self):
        joy_msg = Joy()

        joy_msg.buttons = [0] * 18
        joy_msg.buttons[0] = self.dualsense.state.cross
        joy_msg.buttons[1] = self.dualsense.state.circle
        joy_msg.buttons[2] = self.dualsense.state.triangle
        joy_msg.buttons[3] = self.dualsense.state.square
        joy_msg.buttons[4] = self.dualsense.state.L1
        joy_msg.buttons[5] = self.dualsense.state.R1
        joy_msg.buttons[6] = self.dualsense.state.L2Btn
        joy_msg.buttons[7] = self.dualsense.state.R2Btn
        joy_msg.buttons[8] = self.dualsense.state.L3
        joy_msg.buttons[9] = self.dualsense.state.R3
        joy_msg.buttons[10] = self.dualsense.state.ps
        joy_msg.buttons[11] = self.dualsense.state.share
        joy_msg.buttons[12] = self.dualsense.state.options
        joy_msg.buttons[13] = self.dualsense.state.DpadUp
        joy_msg.buttons[14] = self.dualsense.state.DpadDown
        joy_msg.buttons[15] = self.dualsense.state.DpadLeft
        joy_msg.buttons[16] = self.dualsense.state.DpadRight
        joy_msg.buttons[17] = self.dualsense.state.touchBtn

        joy_msg.axes = [0.0] * 6
        joy_msg.axes[0] = (-1 * self.dualsense.state.LX) / 128.0
        joy_msg.axes[1] = (-1 * self.dualsense.state.LY) / 128.0
        joy_msg.axes[2] = (-1 * self.dualsense.state.RX) / 128.0
        joy_msg.axes[3] = (-1 * self.dualsense.state.RY) / 128.0
        joy_msg.axes[4] = self.dualsense.state.L2 / 255.0
        joy_msg.axes[5] = self.dualsense.state.R2 / 255.0

        joy_msg.axes = [0.0 if abs(val) < self.deadzone else val for val in joy_msg.axes]

        if self.prev_msg.axes == joy_msg.axes and self.prev_msg.buttons == joy_msg.buttons:
            self.prev_msg = joy_msg
            return

        self.prev_msg = joy_msg
        self.joy_pub.publish(joy_msg)

    def main_loop(self):
        if self.node_state == 0:
            try:
                self.dualsense.init()
            except Exception:
                self.get_logger().warn("Cannot initialize controller!")
            else:
                self.dualsense.light.setColorI(0, 255, 0)
                self.node_state = 1

        elif self.node_state == 1:
            try:
                self.get_logger().info("DS5_Ros is alive!", throttle_duration_sec=2.0)
                self.joy_publish()
            except Exception as e:
                self.get_logger().error(f"Controller error: {e}")
                print(traceback.format_exc())
                self.node_state = 0
        if not self.dualsense.connected:
            self.node_state = 0


def main(args=None):
    rclpy.init(args=args)
    node = Ds5Ros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down DS5 node")
    finally:
        node.dualsense.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
