"""Run the real ROS bridge against a virtual COMMAND40/STATE32 MCU (no hardware)."""
import math
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rm_interfaces.msg import GimbalCmd, GimbalStatus, NavCmd
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def state(total, valid):
    b = bytearray(32)
    b[0], b[1] = 0x5A, 61
    # HI91 Euler angles are degrees; teammate Encode forwards them unchanged.
    struct.pack_into("<fff", b, 2, math.degrees(0.05), math.degrees(-0.1), 179)
    struct.pack_into("<HH", b, 26, total & 65535, valid & 65535)
    struct.pack_into("<H", b, 30, crc16(b[:30]))
    return b


def main():
    project = Path(__file__).resolve().parents[3]
    binary = project / ".cache/communication_install/rm_hw_bridge/lib/rm_hw_bridge/hw_bridge_node"
    logdir = project / "docs/validation_communication_20261009"
    logdir.mkdir(parents=True, exist_ok=True)
    master, slave = os.openpty()
    os.set_blocking(master, False)
    rclpy.init()
    node = rclpy.create_node("teammate_pty_test")
    gimbal_pub = node.create_publisher(GimbalCmd, "/gimbal_cmd", qos_profile_sensor_data)
    nav_pub = node.create_publisher(NavCmd, "/nav_cmd", qos_profile_sensor_data)
    statuses, imus, joints, links = [], [], [], []
    subscriptions = [
        node.create_subscription(GimbalStatus, "/gimbal_status", statuses.append, qos_profile_sensor_data),
        node.create_subscription(Imu, "/imu/data", imus.append, qos_profile_sensor_data),
        node.create_subscription(JointState, "/joint_states", joints.append, qos_profile_sensor_data),
        node.create_subscription(String, "/hw_bridge/link_status", links.append, 10),
    ]
    log = (logdir / "pty_bridge.log").open("w")
    proc = subprocess.Popen([str(binary), "--ros-args", "-p", f"serial_device:={os.ttyname(slave)}",
        "-p", "allow_shoot:=true"], stdout=log, stderr=log)
    try:
        wire, frames = bytearray(), []
        total = valid = 65500
        next_state = next_cmd = time.monotonic() + 0.5
        start = time.monotonic()
        moving, stopped, wrapped = False, False, False
        first_delta_seen = False
        while time.monotonic() - start < 5:
            now = time.monotonic()
            elapsed = now - start
            assert proc.poll() is None, "Bridge process exited"
            if now >= next_state and elapsed < 3.5:
                packet = state(total, valid)
                # Noise and fragmented delivery exercise the actual serial RX parser.
                os.write(master, b"noise" + packet[:7])
                time.sleep(0.002)
                os.write(master, packet[7:])
                next_state = now + 0.1
            if now >= next_cmd and elapsed < 2.5:
                command = GimbalCmd()
                command.mode = 1
                command.target_yaw = math.radians(-179)
                command.target_pitch = 0.1
                command.fire_control = 1
                nav = NavCmd()
                nav.linear_x, nav.linear_y = 0.7, -0.2
                gimbal_pub.publish(command)
                nav_pub.publish(nav)
                next_cmd = now + 0.025
            if select.select([master], [], [], 0)[0]:
                try:
                    wire.extend(os.read(master, 4096))
                except BlockingIOError:
                    pass
            while len(wire) >= 40:
                assert wire[0] == 0xA5, "Unexpected old SP/NAVI frame header"
                packet = bytes(wire[:40])
                del wire[:40]
                total = (total + 1) & 65535
                assert crc16(packet[:38]) == struct.unpack_from("<H", packet, 38)[0]
                valid = (valid + 1) & 65535
                frames.append(packet)
                assert packet[22:25] == b"\0\0\0" and packet[29] == 0
                yaw_error, pitch_error = struct.unpack_from("<ff", packet, 9)
                vx, vy = struct.unpack_from("<ff", packet, 30)
                if 1.0 < elapsed < 2.5 and abs(vx - 0.7) < 1e-5:
                    current_pitch, current_yaw = struct.unpack_from("<ff", packet, 1)
                    assert abs(current_pitch - math.degrees(-0.1)) < 0.01
                    assert abs(current_yaw - 179) < 0.01
                    assert abs(vy + 0.2) < 1e-5
                    assert abs(yaw_error - 2) < 0.01
                    assert abs(pitch_error - 5) < 0.01
                    assert packet[21] == 1
                    moving = True
                if elapsed > 3:
                    assert vx == vy == yaw_error == pitch_error == packet[21] == 0
                    stopped = True
            rclpy.spin_once(node, timeout_sec=0.005)
            for status in statuses[-1:]:
                assert abs(status.gimbal_yaw - 179) < 0.01
                assert abs(status.gimbal_pitch - math.degrees(-0.1)) < 0.01
                assert not status.referee_data_valid and not status.mode_valid
                assert not status.ballistic_data_valid and not status.rates_valid
                assert math.isnan(status.bullet_speed) and status.detect_color == 1
                wrapped |= status.rx_count < 65500
            for link in links[-1:]:
                fields = dict(part.split("=") for part in link.data.split())
                delta = int(fields["delta_valid"])
                assert 0 <= delta < 100, "Counter delta failed around uint16 wrap"
                first_delta_seen |= delta > 0
        assert moving and stopped and wrapped and first_delta_seen
        assert len(frames) > 100 and statuses and imus and joints and links
        assert abs(joints[-1].position[0] - math.radians(179)) < 1e-6
        assert abs(joints[-1].position[1] + 0.1) < 1e-6
        assert imus[-1].angular_velocity_covariance[0] == -1
        # Quaternion must convert incoming degrees to radians exactly once.
        roll, pitch, yaw = 0.05, -0.1, math.radians(179)
        expected_w = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        assert abs(imus[-1].orientation.w - expected_w) < 1e-6
        print(f"PASS: real ROS/PTY bridge, {len(frames)} COMMAND40 frames, STATE32 feedback, radians/degrees, counters and stale-command zeroing")
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
        del subscriptions


if __name__ == "__main__":
    main()
