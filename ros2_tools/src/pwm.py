#!/usr/bin/env python3
import json
import time

import Jetson.GPIO as GPIO
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from ros2_tools.srv import SERVO


_DUTY_CYCLE_MAP = {
    0: 6.0,
    1: 8.5,
}


class AirdropPwmNode(Node):
    def __init__(self) -> None:
        super().__init__("airdrop_pwm_node")

        self.declare_parameter("service", "/drone/servo_control")
        self.declare_parameter("command_topic", "/drone/airdrop_cmd")
        self.declare_parameter("pwm_pin", 13)
        self.declare_parameter("frequency", 50.0)
        self.declare_parameter("release_command", 1)
        self.declare_parameter("neutral_command", 0)
        self.declare_parameter("pulse_hold_sec", 1.0)
        self.declare_parameter("auto_return_neutral", False)

        self._pwm_pin = int(self.get_parameter("pwm_pin").value)
        self._frequency = float(self.get_parameter("frequency").value)
        self._release_command = int(self.get_parameter("release_command").value)
        self._neutral_command = int(self.get_parameter("neutral_command").value)
        self._pulse_hold_sec = float(self.get_parameter("pulse_hold_sec").value)
        self._auto_return_neutral = bool(self.get_parameter("auto_return_neutral").value)
        service_name = self.get_parameter("service").value
        command_topic = self.get_parameter("command_topic").value

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pwm_pin, GPIO.OUT)

        self._pwm = GPIO.PWM(self._pwm_pin, self._frequency)
        self._current_duty_cycle = None
        self._set_command(self._neutral_command)

        self._cb_group = ReentrantCallbackGroup()
        self._service = self.create_service(
            SERVO,
            service_name,
            self._on_service_command,
            callback_group=self._cb_group,
        )
        self._command_sub = self.create_subscription(
            String,
            command_topic,
            self._on_airdrop_command,
            10,
        )

        self.get_logger().info(
            f"airdrop pwm ready: topic={command_topic}, service={service_name}, "
            f"pin={self._pwm_pin}, freq={self._frequency:.1f}Hz"
        )

    def _on_service_command(self, request: SERVO.Request, response: SERVO.Response):
        success, message = self._apply_command(int(request.command))
        response.success = success
        response.message = message
        return response

    def _on_airdrop_command(self, msg: String) -> None:
        action = msg.data.strip()
        try:
            payload = json.loads(msg.data)
            action = str(payload.get("action", action))
        except json.JSONDecodeError:
            pass

        if action in ("release", "release_placeholder", "drop", "airdrop"):
            self._apply_command(self._release_command)
        elif action in ("reset", "neutral", "close"):
            self._apply_command(self._neutral_command)
        else:
            self.get_logger().warn(f"ignored unknown airdrop action: {action}")

    def _apply_command(self, command: int) -> tuple[bool, str]:
        duty_cycle = _DUTY_CYCLE_MAP.get(command)
        if duty_cycle is None:
            message = f"Unknown servo command {command}; expected one of {list(_DUTY_CYCLE_MAP)}"
            self.get_logger().warn(message)
            return False, message

        self._set_duty_cycle(duty_cycle)
        self.get_logger().info(f"Applied servo command {command} (duty {duty_cycle:.2f}%)")
        if self._pulse_hold_sec > 0.0:
            time.sleep(self._pulse_hold_sec)

        if self._auto_return_neutral and command != self._neutral_command:
            self._set_command(self._neutral_command)
            self.get_logger().info(f"Returned servo to neutral command {self._neutral_command}")

        return True, f"Applied servo command {command} (duty {duty_cycle:.2f}%)"

    def _set_command(self, command: int) -> None:
        duty_cycle = _DUTY_CYCLE_MAP[command]
        self._set_duty_cycle(duty_cycle)

    def _set_duty_cycle(self, duty_cycle: float) -> None:
        if self._current_duty_cycle is None:
            self._pwm.start(duty_cycle)
        else:
            self._pwm.ChangeDutyCycle(duty_cycle)
        self._current_duty_cycle = duty_cycle

    def destroy_node(self) -> bool:
        if self._pwm is not None:
            self._pwm.stop()
            self._pwm = None
        GPIO.cleanup(self._pwm_pin)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirdropPwmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down airdrop pwm")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
