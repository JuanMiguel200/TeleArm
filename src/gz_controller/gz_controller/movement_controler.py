#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff
from ardupilot_msgs.msg import GlobalPosition
import time

COPTER_MODE_GUIDED = 4
COPTER_TAKEOFF_ALT = 5.0

class CopterMove(Node):
    def __init__(self):
        super().__init__('use_ap_services')
        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._client_arm = self.create_client(ArmMotors, self._arm_topic)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        self.declare_parameter("takeoff_topic", "/ap/experimental/takeoff")
        self._takeoff_topic = self.get_parameter("takeoff_topic").get_parameter_value().string_value
        self._client_takeoff = self.create_client(Takeoff, self._takeoff_topic)
        while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')
      
    #Funtions usage starts    
    
    #arm
    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    #cambio de modo
    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        assert mode in [COPTER_MODE_GUIDED]
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode and self.get_clock().now() - start < timeout:
            result = self.switch_mode(desired_mode)
            # Handle successful switch or the case that the vehicle is already in expected mode
            is_in_desired_mode = result.status or result.curr_mode == desired_mode
            time.sleep(1)

        return is_in_desired_mode
    
    #takeoff

    def takeoff(self, alt):
        req = Takeoff.Request()
        assert alt in [COPTER_TAKEOFF_ALT]
        req.alt = alt
        future = self._client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def takeoff_with_timeout(self, desired_alt: int, timeout: rclpy.duration.Duration):
        """Try to takeoff. Returns true on success, or false if takeoff fails or times out."""
        is_in_desired_alt = False
        start = self.get_clock().now()
        while not is_in_desired_alt and self.get_clock().now() - start < timeout:
            result = self.takeoff(desired_alt)
            # Handle successful switch or the case that the vehicle is already in expected mode
            is_in_desired_alt = result.status or result.curr_mode == desired_alt
            time.sleep(1)

        return is_in_desired_alt

def main(args=None):
    rclpy.init(args=args)
    node = CopterMove()
    try:
        # Block till armed
        if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
            raise RuntimeError("Unable to arm")

        # Block till in guided
        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")
        
        # Block till takeoff
        if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")
        
        
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()