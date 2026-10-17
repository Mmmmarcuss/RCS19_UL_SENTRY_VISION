"""Exercise the actual ROS V2 bridge on PTYs; never opens physical serial ports.

Source the ROS installation and the freshly built project setup.bash first.
Optional: RM_HW_BRIDGE_TEST_ROOT redirects outputs; BRIDGE_TEST_BINARY selects the node.
"""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import tty

os.environ.setdefault('ROS_DOMAIN_ID', '116')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import qos_profile_sensor_data as qos
from rm_interfaces.msg import GimbalCmd, GimbalStatus, ImuTelemetry, NavCmd
from sensor_msgs.msg import Imu


def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0x8408 if c & 1 else 0)
    return c


def frame(kind, seq, boot, payload):
    b = struct.pack('<BBBBHHI', 0xA6, 0x6A, 2, kind, len(payload), seq & 65535, boot) + payload
    return b + struct.pack('<H', crc16(b))


def telemetry(seq, boot, flags):
    stamp = (seq * 20000) & 0xFFFFFFFF if flags & 4 else 0
    g = struct.pack('<BBHIffffff', 1, flags, 1, stamp, 0.05, -0.1, math.radians(179), 0.1, 0.2, 0.3)
    c = struct.pack('<BBHIfff', 2, flags & ~2, 1, stamp, 0.02, -0.03, 0.04)
    link = struct.pack('<IHBBHH', 0, 40000, 0, 0, 65535, 0)
    return frame(1, seq, boot, g) + frame(2, seq, boot, c) + frame(0x81, seq, boot, link)


def drain(fd, wire, frames):
    while True:
        try:
            b = os.read(fd, 4096)
        except (BlockingIOError, OSError):
            break
        if not b:
            break
        wire.extend(b)
    while len(wire) >= 14:
        assert wire[:3] == b'\xa6\x6a\x02', f'Invalid output header: {wire[:12].hex()}'
        length, seq, boot = struct.unpack_from('<HHI', wire, 4)
        if len(wire) < length + 14:
            break
        b = bytes(wire[:length + 14])
        del wire[:length + 14]
        assert crc16(b[:-2]) == struct.unpack_from('<H', b, len(b)-2)[0]
        frames.append((time.monotonic(), b[3], seq, boot, b[12:-2]))


def scenario(project, readonly):
    output = Path(os.environ.get('RM_HW_BRIDGE_TEST_ROOT', project / '.cache'))
    logdir = output / 'v2_validation'
    logdir.mkdir(parents=True, exist_ok=True)
    master, slave = os.openpty()
    tty.setraw(slave)  # Disable PTY echo before the bridge has opened the slave.
    os.set_blocking(master, False)
    node = rclpy.create_node('v2_pty_probe')
    gpub = node.create_publisher(GimbalCmd, '/gimbal_cmd', qos)
    npub = node.create_publisher(NavCmd, '/nav_cmd', qos)
    statuses, imus, bases, raw = [], [], [], []
    subscriptions = [
        node.create_subscription(GimbalStatus, '/gimbal_status', statuses.append, qos),
        node.create_subscription(Imu, '/imu/data', imus.append, qos),
        node.create_subscription(Imu, '/chassis/imu/data', bases.append, qos),
        node.create_subscription(ImuTelemetry, '/hw_bridge/imu_telemetry', raw.append, qos),
    ]
    candidates = [output / 'v2_install/rm_hw_bridge/lib/rm_hw_bridge/hw_bridge_node',
                  project / 'install/rm_hw_bridge/lib/rm_hw_bridge/hw_bridge_node',
                  project / 'install/lib/rm_hw_bridge/hw_bridge_node']
    selected = os.environ.get('BRIDGE_TEST_BINARY')
    binary = Path(selected) if selected else next((p for p in candidates if p.is_file()), candidates[0])
    assert binary.is_file(), f'Build rm_hw_bridge first or set BRIDGE_TEST_BINARY: {binary}'
    device = logdir / ('readonly_serial' if readonly else 'tx_serial')
    device.unlink(missing_ok=True)
    device.symlink_to(os.ttyname(slave))
    log = (logdir / ('pty_readonly.log' if readonly else 'pty_tx.log')).open('w')
    args = [str(binary), '--ros-args', '--params-file', str(project / 'src/rm_hw_bridge/config/params.yaml'),
            '-p', f'serial_device:={device}', '-p', f'read_only:={str(readonly).lower()}',
            '-p', 'allow_control:=true', '-p', 'allow_nav:=true']
    proc = subprocess.Popen(args, stdout=log, stderr=log)
    wire, frames = bytearray(), []
    started = time.monotonic()
    seq = 0
    next_rx = next_cmd = started
    client = node.create_client(SetParameters, '/rm_hw_bridge/set_parameters')
    future = None
    reconnected = False
    stale_seen = False
    try:
        while time.monotonic() - started < 5.5:
            now = time.monotonic()
            elapsed = now-started
            assert proc.poll() is None, 'Bridge exited; inspect PTY log'
            if elapsed > 1.8 and not reconnected:
                oldmaster, oldslave = master, slave
                master, slave = os.openpty()
                tty.setraw(slave)
                os.set_blocking(master, False)
                device.unlink()
                device.symlink_to(os.ttyname(slave))
                os.close(oldmaster); os.close(oldslave)
                reconnected = True
            if now >= next_rx and (elapsed < 4.7 or elapsed > 5.2):
                flags = 3 if readonly or 2.1 <= elapsed < 2.8 else 15
                boot = 99 if elapsed >= 3.2 else 0x12345678
                packet = telemetry(seq, boot, flags)
                if seq == 0:
                    bad = bytearray(packet[:46]); bad[25] ^= 1
                    os.write(master, b'noise' + bad)
                os.write(master, packet[:7])
                time.sleep(0.002)
                os.write(master, packet[7:])
                seq += 1
                next_rx = now+0.02
            if now >= next_cmd and elapsed < 3.8:
                command = GimbalCmd()
                command.mode = 1; command.target_yaw = math.radians(-179)
                command.target_pitch = 0.1; command.fire_control = 1
                nav = NavCmd(); nav.linear_x = 0.4; nav.linear_y = -0.2; nav.angular_z = 0.8
                gpub.publish(command); npub.publish(nav)
                next_cmd = now+0.04
            if future is None and elapsed > 1 and client.service_is_ready():
                request = SetParameters.Request()
                request.parameters = [Parameter('read_only', value=not readonly).to_parameter_msg()]
                future = client.call_async(request)
            rclpy.spin_once(node, timeout_sec=0.002)
            drain(master, wire, frames)
            if 4.9 < elapsed < 5.2 and statuses and not statuses[-1].pose_valid:
                stale_seen = True
        assert statuses and imus and bases and raw, 'Missing decoded ROS topics'
        assert future is not None and future.done(), 'No parameter response'
        assert not future.result().results[0].successful, 'read_only must be immutable'
        assert all(not s.referee_data_valid and not s.mode_valid and not s.ballistic_data_valid and
                   not s.rates_valid and s.detect_color == 255 for s in statuses)
        assert stale_seen and any(not s.pose_valid and math.isnan(s.gimbal_yaw) for s in statuses), 'No stale status'
        assert {r.sensor_id for r in raw} == {1, 2}
        if readonly:
            assert not frames and not wire, 'Receive-only emitted UART bytes'
            assert all(not s.pose_valid for s in statuses)
            assert all(i.orientation_covariance[0] == -1 and i.angular_velocity_covariance[0] == -1 for i in imus)
            assert any(r.flags == 3 and r.sample_time_us == 0 and abs(r.gz-0.3) < 1e-6 for r in raw)
        else:
            assert frames and not wire
            assert {f[1] for f in frames} == {0x10, 0x11}
            active = [f for f in frames if f[1] == 0x10 and f[4][14] & 1]
            navs = [f for f in frames if f[1] == 0x11 and f[4][10] & 1]
            assert active and navs, 'No enabled output with validated fixture'
            assert all(f[4][14] == 3 for f in active), 'Friction/fire enabled unexpectedly'
            assert all(abs(struct.unpack_from('<f', f[4], 16)[0]-math.radians(2)) < 1e-5 for f in active)
            assert all(abs(struct.unpack_from('<f', f[4], 20)[0]-math.radians(5)) < 1e-5 for f in active)
            assert all(struct.unpack_from('<f', f[4], 20)[0] == 0 for f in navs), 'Unsupported wz transmitted'
            assert all(f[4][14] == 0 for f in frames if f[1] == 0x10 and 2.3 < f[0]-started < 2.7)
            assert all((f[4][14] if f[1] == 0x10 else f[4][10]) == 0
                       for f in frames if f[0]-started > 4.1), 'Stale command replayed'
            assert any(f[3] == 99 for f in frames), 'MCU boot change not followed'
            assert any(s.pose_valid and abs(s.gimbal_yaw-179) < 1e-4 for s in statuses)
            assert any(abs(i.angular_velocity.z-0.3) < 1e-6 for i in imus)
        before_exit = len(frames)
        proc.send_signal(signal.SIGINT); proc.wait(timeout=5)
        drain(master, wire, frames)
        if readonly:
            assert not frames and not wire, 'Receive-only sent bytes during exit'
        else:
            assert len(frames) >= before_exit+2, 'No final AIM/NAV STOP batch'
            assert frames[-2][1] == 0x10 and frames[-2][4][14] == 0
            assert frames[-1][1] == 0x11 and frames[-1][4][10] == 0
        assert proc.returncode == 0, f'Bridge exit code {proc.returncode}'
        log.flush()
        assert (logdir / ('pty_readonly.log' if readonly else 'pty_tx.log')).read_text().count('Serial opened:') >= 2
        print('PASS:', 'read-only startup/runtime/exit = zero bytes' if readonly else
              'V2 ROS PTY decode, two IMUs, radian TX, flags/permissions/timeouts/reboot/reconnect', flush=True)
    finally:
        if proc.poll() is None:
            proc.terminate(); proc.wait(timeout=5)
        log.close(); node.destroy_node(); os.close(master); os.close(slave)
        device.unlink(missing_ok=True)
        del subscriptions


def main():
    project = Path(__file__).resolve().parents[3]
    rclpy.init()
    try:
        scenario(project, True)
        scenario(project, False)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
