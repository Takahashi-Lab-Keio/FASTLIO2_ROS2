"""ROS integration tests; use an isolated ROS domain (no robot processes)."""
import contextlib
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from fastlio2_interfaces.msg import LocalizationState
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


@contextlib.contextmanager
def ros_nodes(tmp_path, executables, expect_exit=False):
    # No hardware launch, map server, controller or velocity publisher is used.
    os.environ['ROS_DOMAIN_ID'] = '193'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    context = rclpy.context.Context()
    rclpy.init(context=context)
    probe = rclpy.create_node('prediction_test_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    processes, logs = [], []
    try:
        for executable, arguments in executables:
            log = (tmp_path / f'{executable}.log').open('w')
            logs.append(log)
            path = Path(get_package_prefix('fastlio2')) / 'lib/fastlio2' / executable
            processes.append(subprocess.Popen(
                [str(path), '--ros-args', '-r', '__ns:=/fastlio2', *arguments],
                stdout=log, stderr=subprocess.STDOUT))
        yield probe
        if expect_exit:
            assert all(p.wait(timeout=5) == 0 for p in processes)
        else:
            assert all(p.poll() is None for p in processes), [p.returncode for p in processes]
    finally:
        for p in processes:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait(timeout=5)
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=context)
        for log in logs:
            log.close()
        for path in tmp_path.glob('*.log'):
            print(path.name, path.read_text()[-4000:])


def spin_for(node, duration):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        node.executor.spin_once(timeout_sec=0.005)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def test_tf_projection_invalidation_and_no_retimestamping(tmp_path):
    with ros_nodes(tmp_path, [('nav_tf_node', [])]) as node:
        pub = node.create_publisher(Odometry, '/fastlio2/predicted_odom', 10)
        state_pub = node.create_publisher(LocalizationState, '/fastlio2/localizer/localization_state',
                                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transforms = []
        node.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(m.transforms), 100)
        spin_for(node, 0.6)
        state = LocalizationState(valid=True)
        state.map_to_local.header.frame_id = 'map'
        state.map_to_local.child_frame_id = 'fastlio_odom'
        state.map_to_local.header.stamp = node.get_clock().now().to_msg()
        state.map_to_local.transform.translation.z = 0.3
        state.map_to_local.transform.rotation.w = 1.0
        state_pub.publish(state)
        spin_for(node, 0.1)
        pose = Odometry()
        pose.header.frame_id = 'fastlio_odom'
        pose.child_frame_id = 'fastlio_body'
        pose.pose.pose.position.x = 2.0
        pose.pose.pose.position.z = -0.12
        pose.pose.pose.orientation.x = 0.1
        pose.pose.pose.orientation.w = (1.0 - 0.01) ** 0.5
        pose.header.stamp = node.get_clock().now().to_msg()
        pub.publish(pose)
        spin_for(node, 0.15)
        by_child = {t.child_frame_id: t for t in transforms}
        assert set(by_child) == {'base_footprint', 'odom', 'fastlio_odom', 'fastlio_body'}
        nav = by_child['base_footprint']
        assert nav.header.stamp == pose.header.stamp
        assert nav.transform.translation.z == 0.0
        assert nav.transform.rotation.x == nav.transform.rotation.y == 0.0
        assert by_child['fastlio_body'].transform.translation.z == -0.12
        assert by_child['fastlio_body'].transform.rotation.x == pytest.approx(0.1)
        count = len(transforms)
        spin_for(node, 0.2)
        assert len(transforms) == count  # no timer silently refreshes TF
        pub.publish(pose)  # old source pose
        spin_for(node, 0.1)
        assert len(transforms) == count
        state_pub.publish(LocalizationState(valid=False))
        spin_for(node, 0.1)
        transforms.clear()
        pose.header.stamp = node.get_clock().now().to_msg()
        pub.publish(pose)
        spin_for(node, 0.1)
        assert {t.child_frame_id for t in transforms} == {'base_footprint', 'fastlio_body'}


@pytest.mark.parametrize("compact", [False, True])
def test_real_lio_node_predicts_past_delayed_scans_and_stops_on_dropout(tmp_path, compact):
    config = Path(__file__).resolve().parents[1] / 'config/jt128.yaml'
    args = ['-p', f'config_path:={config}', '-p', 'allow_missing_point_time:=true',
            '-p', 'startup_continuous_input_seconds:=0.0',
            '-p', 'enable_imu_prediction:=true', '-p', 'publish_tf:=false',
            '-p', 'body_frame_override:=fastlio_body']
    with ros_nodes(tmp_path, [('lio_node', args)]) as node:
        imu_pub = node.create_publisher(Imu, '/lidar_imu', 200)
        cloud_pub = node.create_publisher(PointCloud2, '/lidar_points', 10)
        predicted, corrected, diagnostics, processing = [], [], [], []
        node.create_subscription(Odometry, '/fastlio2/predicted_odom', lambda m: predicted.append(m), 200)
        node.create_subscription(Odometry, '/fastlio2/lio_odom', lambda m: corrected.append(m), 100)
        node.create_subscription(DiagnosticArray, '/fastlio2/prediction/diagnostics',
                                 lambda m: diagnostics.extend(m.status), 100)
        node.create_subscription(DiagnosticArray, '/fastlio2/processing/diagnostics',
                                 lambda m: processing.extend(m.status), 100)
        spin_for(node, 0.5)
        # Three orthogonal planes; enough non-degenerate points for the actual
        # estimator, deskewer, ICP Jacobians and kd-tree initialization.
        points = []
        for i in range(-12, 13):
            for j in range(-8, 9):
                a, b = i * 0.18, j * 0.18
                points.extend([(4.0, a, b), (a, 4.0, b), (a, b, -1.0)])
        fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud = create_cloud(Header(frame_id='hesai_lidar'), fields, [(*p, 1.0) for p in points])
        if compact:
            fields += [PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1),
                       PointField(name='preprocessing_stride', offset=24, datatype=PointField.UINT16, count=1)]
        imu = Imu()
        imu.header.frame_id = 'hesai_lidar'
        imu.linear_acceleration.z = 9.81
        begin = time.monotonic()
        next_imu, next_cloud = begin, begin + 0.35
        while time.monotonic() - begin < 5.0:
            now = time.monotonic()
            if now >= next_imu:
                imu.header.stamp = node.get_clock().now().to_msg()
                imu_pub.publish(imu)
                next_imu = now + 0.005
            if now >= next_cloud:
                # Emulate 250 ms scan/processing latency, while IMU stays fresh.
                cloud.header.stamp = (node.get_clock().now() - Duration(seconds=0.25)).to_msg()
                if compact:
                    # Same retained points as raw/6, but real per-point timing
                    # and scan end; input remains 250 ms late at its END.
                    stamp = node.get_clock().now() - Duration(seconds=0.35)
                    start = stamp.nanoseconds * 1e-9
                    selected = points[::6]
                    records = [(*p, 1.0, start + .099*i/len(selected), 6)
                               for i,p in enumerate(selected)]
                    records.append((float('nan'), float('nan'), float('nan'), 0.0, start+.1, 6))
                    cloud = create_cloud(Header(frame_id='hesai_lidar', stamp=stamp.to_msg()), fields, records)
                cloud_pub.publish(cloud)
                next_cloud = now + 0.1
            node.executor.spin_once(timeout_sec=0.001)
        received = [dict((v.key, float(v.value)) for v in d.values)
                    for d in processing if d.message == 'received']
        completed = [dict((v.key, float(v.value)) for v in d.values)
                     for d in processing if d.message == 'processed']
        assert len(received) > 10 and len(completed) > 10
        assert all(d['preprocessing_stride'] == (6 if compact else 0) for d in received)
        assert all(d['estimator_ms'] >= 0 and d['output_ms'] >= 0 and
                   d['queue_wait_ms'] >= 0 and d['queue_drops'] == 0 for d in completed)
        assert len(corrected) > 10
        assert len(predicted) > 150
        assert seconds(predicted[-1].header.stamp) - seconds(corrected[-1].header.stamp) > 0.15
        good = [d for d in diagnostics if d.message == 'predicting']
        assert good
        timings = sorted(float(dict((v.key, v.value) for v in d.values)['processing_ms']) for d in good)
        print('prediction_callback_p99_ms', timings[int(0.99 * (len(timings) - 1))])
        assert timings[int(0.99 * (len(timings) - 1))] < 10.0
        # LiDAR continues to be absent; IMU prediction must stop after the bound.
        spin_for(node, 0.15)
        count = len(predicted)
        spin_for(node, 0.15)
        assert len(predicted) == count
        assert any(d.message == 'imu_stale' for d in diagnostics[-4:])
