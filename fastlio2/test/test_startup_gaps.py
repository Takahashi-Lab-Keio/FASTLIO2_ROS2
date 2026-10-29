"""Exercise real LIO initialization with controlled sensor time and no hardware."""
import contextlib
import math
from pathlib import Path
import time

import pytest
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from test_prediction_nodes import ros_nodes, seconds, spin_for


def stamp(t):
    return Time(nanoseconds=round(t * 1e9)).to_msg()


def wait_for(node, predicate):
    end = time.monotonic() + 5.0
    while not predicate() and time.monotonic() < end:
        spin_for(node, 0.01)
    assert predicate()


class Stream:
    def __init__(self, node):
        self.node = node
        self.imu_pub = node.create_publisher(Imu, '/lidar_imu', 200)
        self.cloud_pub = node.create_publisher(PointCloud2, '/lidar_points', 10)
        self.clock_pub = node.create_publisher(Clock, '/clock', 10)
        self.odom, self.predicted, self.tf, self.processing = [], [], [], []
        self.subs = [
            node.create_subscription(Odometry, '/fastlio2/lio_odom', self.odom.append, 100),
            node.create_subscription(Odometry, '/fastlio2/predicted_odom', self.predicted.append, 100),
            node.create_subscription(TFMessage, '/tf', lambda m: self.tf.extend(m.transforms), 100),
            node.create_subscription(DiagnosticArray, '/fastlio2/processing/diagnostics',
                                     lambda m: self.processing.extend(m.status), 100),
        ]
        self.points = []
        for i in range(-12, 13):
            for j in range(-8, 9):
                a, b = i * 0.18, j * 0.18
                self.points.extend([(4.0, a, b), (a, 4.0, b), (a, b, -1.0)])
        self.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                       for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        self.fields.append(PointField(name='timestamp', offset=16,
                                      datatype=PointField.FLOAT64, count=1))
        wait_for(node, lambda: self.imu_pub.get_subscription_count() > 0 and
                 self.cloud_pub.get_subscription_count() > 0 and
                 self.clock_pub.get_subscription_count() > 0)
        self.last_imu = 10.0 - 0.005
        self.clock_pub.publish(Clock(clock=stamp(10.0)))
        spin_for(node, 0.1)

    def imu(self, t, gyro=0.0):
        msg = Imu(header=Header(frame_id='hesai_lidar', stamp=stamp(t)))
        msg.linear_acceleration.z = 9.81
        msg.angular_velocity.z = gyro
        self.imu_pub.publish(msg)
        self.last_imu = t
        self.clock_pub.publish(Clock(clock=stamp(t)))

    def imus_until(self, t, gyro=0.0):
        while self.last_imu < t - 1e-8:
            self.imu(self.last_imu + 0.005, gyro)
            spin_for(self.node, 0.001)

    def cloud(self, start, duration=0.08):
        records = [(*p, 1.0, start + duration*i/(len(self.points)-1))
                   for i, p in enumerate(self.points)]
        self.cloud_pub.publish(create_cloud(
            Header(frame_id='hesai_lidar', stamp=stamp(start)), self.fields, records))
        spin_for(self.node, 0.1)

    def frame(self, start, gyro=0.0):
        self.imus_until(start + 0.085, gyro)
        before = self.completed()
        self.cloud(start)
        wait_for(self.node, lambda: self.completed() > before)

    def completed(self):
        return sum(s.message in ('initializing', 'processed',
                                 'startup_waiting_for_continuous_input') for s in self.processing)

    def retries(self, sensor='imu'):
        return [s for s in self.processing if s.message == f'startup_waiting_after_{sensor}_gap']

    def assert_no_pose(self):
        assert not self.odom and not self.predicted and not self.tf

    def assert_recovered(self, after):
        wait_for(self.node, lambda: bool(self.odom) and bool(self.predicted) and bool(self.tf))
        assert all(seconds(m.header.stamp) >= after for m in self.odom)
        assert all(seconds(m.header.stamp) >= after for m in self.predicted)
        assert all(seconds(m.header.stamp) >= after for m in self.tf)
        q = self.odom[-1].pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        assert abs(yaw) < 0.01  # no pre-gap gyro bias carried into the new estimator


@contextlib.contextmanager
def startup_node(tmp_path, init_count=20, expect_exit=False, continuous_seconds=0.0):
    config = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/jt128.yaml').read_text())
    config['imu_init_num'] = init_count
    config['startup_continuous_input_seconds'] = continuous_seconds
    config_path = tmp_path / 'startup.yaml'
    config_path.write_text(yaml.safe_dump(config))
    args = ['-p', f'config_path:={config_path}', '-p', 'use_sim_time:=true',
            '-p', 'enable_imu_prediction:=true', '-p', 'publish_tf:=true']
    with ros_nodes(tmp_path, [('lio_node', args)], expect_exit=expect_exit) as node:
        yield Stream(node)


@pytest.mark.parametrize('init_count,gap', [(60, 0.425), (20, 0.616)])
def test_imu_gap_discards_partial_initialization_and_waiting_scan(tmp_path, init_count, gap):
    with startup_node(tmp_path, init_count) as s:
        s.frame(10.02, gyro=0.3)
        s.assert_no_pose()  # IMU_INIT or MAP_INIT, depending on init_count
        # Leave a scan queued waiting for its end-time IMU.
        s.cloud(10.12)
        s.imu(s.last_imu + gap)
        cutoff = s.last_imu
        wait_for(s.node, lambda: len(s.retries()) == 1)
        # A late scan overlapping the gap must not initialize the new estimator.
        s.cloud(cutoff - 0.04)
        s.assert_no_pose()
        before = s.completed()
        s.frame(cutoff + 0.01)
        assert s.completed() == before + 1
        s.assert_no_pose()  # old initialization count/map must not survive
        for i in range(1, 6):
            s.frame(cutoff + 0.01 + 0.1*i)
        s.assert_recovered(cutoff)


def test_repeated_startup_gaps_before_first_cloud_do_not_exit(tmp_path):
    with startup_node(tmp_path) as s:
        s.imus_until(10.05)
        for count, gap in enumerate((0.425, 0.616, 0.300), 1):
            s.imu(s.last_imu + gap)
            wait_for(s.node, lambda: len(s.retries()) == count)
            s.assert_no_pose()
        cutoff = s.last_imu
        assert dict((v.key, v.value) for v in s.retries()[-1].values)['startup_retry_count'] == '3.000000'
        for i in range(4):
            s.frame(cutoff + 0.01 + i*0.1)
        s.assert_recovered(cutoff)


def test_lidar_startup_gap_discards_old_imu_history(tmp_path):
    with startup_node(tmp_path) as s:
        s.frame(10.02, gyro=0.3)
        s.imus_until(11.20, gyro=0.3)
        s.cloud(11.22)
        wait_for(s.node, lambda: len(s.retries('lidar')) == 1)
        s.assert_no_pose()
        for i in range(5):
            s.frame(11.32 + i*0.1)
        s.assert_recovered(11.22)


@pytest.mark.parametrize('sensor', ['imu', 'lidar'])
@pytest.mark.parametrize('continuous_seconds', [0.0, 0.3])
def test_forward_gap_after_first_pose_remains_fatal(tmp_path, sensor, continuous_seconds):
    with startup_node(tmp_path, expect_exit=True, continuous_seconds=continuous_seconds) as s:
        for i in range(4):
            s.frame(10.02 + i*0.1)
        s.assert_recovered(10.0)
        if sensor == 'imu':
            s.imu(s.last_imu + 0.3)
        else:
            s.imus_until(11.62)
            s.cloud(11.62)
        wait_for(s.node, lambda: 'after estimation started; restart lio_node' in
                 (tmp_path / 'lio_node.log').read_text())
        assert not s.retries(sensor)


@pytest.mark.parametrize('sensor', ['imu', 'lidar'])
def test_gap_after_map_initialization_restarts_the_continuous_input_window(tmp_path, sensor):
    with startup_node(tmp_path, continuous_seconds=0.8) as s:
        for i in range(6):
            s.frame(10.02 + i*0.1)
        assert any(d.message == 'startup_waiting_for_continuous_input' for d in s.processing)
        s.assert_no_pose()  # MAPPING is not yet externally committed
        if sensor == 'imu':
            s.imu(s.last_imu + 0.219)  # observed startup failure
            cutoff = s.last_imu
        else:
            s.imus_until(11.7)
            s.cloud(11.72)
            cutoff = 11.72
        wait_for(s.node, lambda: len(s.retries(sensor)) == 1)
        for i in range(7):
            s.frame(cutoff + .01 + i*.1)
        s.assert_no_pose()  # elapsed time before the gap must not count
        for i in range(7, 10):
            s.frame(cutoff + .01 + i*.1)
        s.assert_recovered(cutoff + .8)


@pytest.mark.parametrize('sensor', ['imu', 'lidar'])
def test_clock_regression_remains_fatal_during_startup(tmp_path, sensor):
    with startup_node(tmp_path, expect_exit=True) as s:
        s.frame(10.02)
        s.assert_no_pose()
        if sensor == 'imu':
            # Keep ROS clock monotonic: only the sensor stamp regresses.
            msg = Imu(header=Header(frame_id='hesai_lidar', stamp=stamp(9.0)))
            msg.linear_acceleration.z = 9.81
            s.imu_pub.publish(msg)
        else:
            s.cloud(9.0)
        wait_for(s.node, lambda: 'clock regressed' in (tmp_path / 'lio_node.log').read_text())
        assert not s.retries(sensor)
