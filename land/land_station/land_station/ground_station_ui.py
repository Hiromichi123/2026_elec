#!/usr/bin/env python3
import json
import math
import threading
from dataclasses import dataclass
from typing import Optional

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_tools.msg import LidarPose


FIELD_W_M = 4.0
FIELD_H_M = 5.0
SCREEN_W = 960
SCREEN_H = 760
MARGIN = 70


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    seen: bool = False


class GroundStationNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_station_ui")
        self.command_pub = self.create_publisher(String, "/mission/command", 10)
        self.drone_status_sub = self.create_subscription(String, "/drone/status", self._on_drone_status, 10)
        self.car_status_sub = self.create_subscription(String, "/car/status", self._on_car_status, 10)
        self.drone_pose_sub = self.create_subscription(LidarPose, "/drone/lidar_data", self._on_drone_pose, 10)
        self.car_pose_sub = self.create_subscription(LidarPose, "/carrier/lidar_pose", self._on_car_pose, 10)

        self.drone_pose = Pose2D()
        self.car_pose = Pose2D()
        self.drone_status = "WAITING"
        self.car_status = "WAITING"
        self.last_command = "none"
        self._lock = threading.Lock()

    def publish_start(self, task_id: int) -> None:
        msg = String()
        msg.data = json.dumps({"command": "start", "task": task_id}, ensure_ascii=False)
        self.command_pub.publish(msg)
        with self._lock:
            self.last_command = f"start task {task_id}"
        self.get_logger().info(f"Published mission start: task={task_id}")

    def snapshot(self):
        with self._lock:
            return (
                Pose2D(**self.drone_pose.__dict__),
                Pose2D(**self.car_pose.__dict__),
                self.drone_status,
                self.car_status,
                self.last_command,
            )

    def _on_drone_pose(self, msg: LidarPose) -> None:
        with self._lock:
            self.drone_pose = Pose2D(msg.x, msg.y, msg.z, msg.yaw, True)

    def _on_car_pose(self, msg: LidarPose) -> None:
        with self._lock:
            self.car_pose = Pose2D(msg.x, msg.y, msg.z, msg.yaw, True)

    def _on_drone_status(self, msg: String) -> None:
        with self._lock:
            self.drone_status = self._status_text(msg.data, "phase")

    def _on_car_status(self, msg: String) -> None:
        with self._lock:
            self.car_status = self._status_text(msg.data, "state")

    @staticmethod
    def _status_text(raw: str, key: str) -> str:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return raw[:80]
        main = payload.get(key, "UNKNOWN")
        detail = payload.get("detail", "")
        task = payload.get("task", 0)
        return f"task {task} | {main} | {detail}"


def field_to_screen(x_m: float, y_m: float) -> tuple[int, int]:
    field_w_px = SCREEN_W - 2 * MARGIN
    field_h_px = SCREEN_H - 2 * MARGIN
    sx = MARGIN + int((x_m / FIELD_W_M) * field_w_px)
    sy = SCREEN_H - MARGIN - int((y_m / FIELD_H_M) * field_h_px)
    return sx, sy


def draw_pose(screen, pose: Pose2D, color: tuple[int, int, int], label: str, font) -> None:
    if not pose.seen:
        return
    x, y = field_to_screen(pose.x, pose.y)
    pygame.draw.circle(screen, color, (x, y), 12)
    tip = (
        x + int(math.cos(pose.yaw) * 24),
        y - int(math.sin(pose.yaw) * 24),
    )
    pygame.draw.line(screen, color, (x, y), tip, 3)
    text = font.render(f"{label} ({pose.x:.2f},{pose.y:.2f},{pose.z:.2f})", True, color)
    screen.blit(text, (x + 14, y - 20))


def draw_track(screen) -> None:
    white = (230, 235, 240)
    grid = (70, 80, 88)
    field_rect = pygame.Rect(MARGIN, MARGIN, SCREEN_W - 2 * MARGIN, SCREEN_H - 2 * MARGIN)
    pygame.draw.rect(screen, white, field_rect, 2)

    for x in range(0, 5):
        sx, _ = field_to_screen(float(x), 0.0)
        pygame.draw.line(screen, grid, (sx, MARGIN), (sx, SCREEN_H - MARGIN), 1)
    for y in range(0, 6):
        _, sy = field_to_screen(0.0, float(y))
        pygame.draw.line(screen, grid, (MARGIN, sy), (SCREEN_W - MARGIN, sy), 1)

    track = [(1.5, 2.0), (1.5, 3.5)]
    for i in range(1, 25):
        a = math.pi - math.pi * i / 24.0
        track.append((2.25 + 0.75 * math.cos(a), 3.5 + 0.75 * math.sin(a)))
    track.append((3.0, 2.0))
    for i in range(1, 25):
        a = 0.0 - math.pi * i / 24.0
        track.append((2.25 + 0.75 * math.cos(a), 2.0 + 0.75 * math.sin(a)))
    for a, b in zip(track, track[1:]):
        pygame.draw.line(screen, (35, 35, 35), field_to_screen(*a), field_to_screen(*b), 5)

    for name, pos in {"H": (0.75, 0.75), "A": (1.5, 2.0), "B": (1.5, 3.5), "C": (3.0, 3.5), "D": (3.0, 2.0)}.items():
        sx, sy = field_to_screen(*pos)
        pygame.draw.circle(screen, (255, 210, 80), (sx, sy), 6)
        pygame.draw.circle(screen, (20, 20, 20), (sx, sy), 8, 1)


def ui_loop(node: GroundStationNode) -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("2026 Air-Ground Ground Station")
    font = pygame.font.SysFont("monospace", 20)
    small = pygame.font.SysFont("monospace", 16)
    clock = pygame.time.Clock()

    running = True
    while running and rclpy.ok():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1:
                    node.publish_start(1)
                elif event.key == pygame.K_2:
                    node.publish_start(2)
                elif event.key == pygame.K_ESCAPE:
                    running = False

        drone_pose, car_pose, drone_status, car_status, last_command = node.snapshot()
        screen.fill((18, 22, 26))
        draw_track(screen)
        draw_pose(screen, car_pose, (80, 210, 120), "CAR", small)
        draw_pose(screen, drone_pose, (90, 170, 255), "UAV", small)

        lines = [
            "1: Start airdrop mission    2: Start dynamic landing mission    Esc: quit",
            f"Last command: {last_command}",
            f"Drone: {drone_status}",
            f"Car:   {car_status}",
            "Topics: /mission/command, /drone/status, /car/status, /drone/lidar_data, /carrier/lidar_pose",
        ]
        for i, line in enumerate(lines):
            screen.blit(font.render(line, True, (230, 235, 240)), (24, 18 + i * 26))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GroundStationNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        ui_loop(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
