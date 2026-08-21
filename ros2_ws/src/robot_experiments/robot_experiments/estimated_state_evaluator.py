"""ROS-only adapter for estimated-state measurement against simulator GT."""

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu

from robot_experiments.estimated_state_metrics import evaluate_trajectory
from robot_experiments.estimated_state_metrics import PoseSample
from robot_experiments.estimated_state_metrics import stream_diagnostics


class ImuYawAccumulator:
    """Passively integrate one monotonic IMU yaw-rate stream."""

    def __init__(self, topic):
        self.topic = topic
        self.samples = []
        self.angular_velocity_z = []
        self.angular_velocity_covariance_z = []
        self.last_stamp_ns = None
        self.last_angular_z = None
        self.integrated_yaw = 0.0
        self.rejected_nonfinite = 0
        self.rejected_nonmonotonic = 0

    def observe(self, message):
        stamp_ns = _stamp_ns(message.header.stamp)
        angular_z = float(message.angular_velocity.z)
        covariance_z = float(message.angular_velocity_covariance[8])
        if not math.isfinite(angular_z):
            self.rejected_nonfinite += 1
            return
        if stamp_ns <= 0 or (
            self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns
        ):
            self.rejected_nonmonotonic += 1
            return
        if self.last_stamp_ns is not None:
            delta_sec = (stamp_ns - self.last_stamp_ns) / 1.0e9
            self.integrated_yaw += (
                0.5 * (self.last_angular_z + angular_z) * delta_sec
            )
        self.last_stamp_ns = stamp_ns
        self.last_angular_z = angular_z
        self.angular_velocity_z.append(angular_z)
        self.angular_velocity_covariance_z.append(covariance_z)
        self.samples.append(PoseSample(
            stamp_ns=stamp_ns,
            x=0.0,
            y=0.0,
            yaw=self.integrated_yaw,
            covariance=None,
        ))

    def diagnostics(self):
        finite_covariances = [
            value for value in self.angular_velocity_covariance_z
            if math.isfinite(value)
        ]
        positive_covariances = [
            value for value in finite_covariances if value > 0.0
        ]
        return {
            'topic': self.topic,
            'sample_count': len(self.samples),
            'rejected_nonfinite': self.rejected_nonfinite,
            'rejected_nonmonotonic': self.rejected_nonmonotonic,
            'mean_angular_velocity_z_rad_s': (
                sum(self.angular_velocity_z) / len(self.angular_velocity_z)
                if self.angular_velocity_z else None
            ),
            'endpoint_integrated_yaw_rad': self.integrated_yaw,
            'angular_velocity_z_covariance': {
                'finite_fraction': (
                    len(finite_covariances)
                    / len(self.angular_velocity_covariance_z)
                    if self.angular_velocity_covariance_z else None
                ),
                'positive_fraction': (
                    len(positive_covariances)
                    / len(self.angular_velocity_covariance_z)
                    if self.angular_velocity_covariance_z else None
                ),
                'mean': (
                    sum(finite_covariances) / len(finite_covariances)
                    if finite_covariances else None
                ),
            },
        }


class EstimatedStateEvaluator(Node):
    """Observe estimates and evaluator-only GT without publishing state or TF."""

    def __init__(self):
        super().__init__('estimated_state_evaluator')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('max_time_delta_sec', 0.1)
        self.declare_parameter('max_time_offset_sec', 0.2)
        self.declare_parameter('time_offset_step_sec', 0.01)
        self.declare_parameter('report_period_sec', 5.0)
        self.declare_parameter('episode_id', '')
        self.declare_parameter('arm', '')

        output_dir = str(self.get_parameter('output_dir').value).strip()
        if not output_dir:
            raise ValueError('output_dir must be set explicitly')
        self._output_dir = Path(output_dir).expanduser().resolve()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        max_delta_sec = float(self.get_parameter('max_time_delta_sec').value)
        max_offset_sec = float(
            self.get_parameter('max_time_offset_sec').value)
        offset_step_sec = float(
            self.get_parameter('time_offset_step_sec').value)
        report_period_sec = float(
            self.get_parameter('report_period_sec').value)
        if not math.isfinite(max_delta_sec) or max_delta_sec < 0.0:
            raise ValueError('max_time_delta_sec must be finite and non-negative')
        if not math.isfinite(max_offset_sec) or max_offset_sec < 0.0:
            raise ValueError('max_time_offset_sec must be finite and non-negative')
        if not math.isfinite(offset_step_sec) or offset_step_sec <= 0.0:
            raise ValueError('time_offset_step_sec must be finite and positive')
        if not math.isfinite(report_period_sec) or report_period_sec <= 0.0:
            raise ValueError('report_period_sec must be finite and positive')
        self._max_time_delta_ns = int(round(max_delta_sec * 1.0e9))
        self._max_time_offset_ns = int(round(max_offset_sec * 1.0e9))
        self._time_offset_step_ns = int(round(offset_step_sec * 1.0e9))
        self._episode_id = str(self.get_parameter('episode_id').value)
        self._arm = str(self.get_parameter('arm').value)

        self._odom_samples = []
        self._wheel_odom_samples = []
        self._lidar_odom_samples = []
        self._imu_data_raw = ImuYawAccumulator('/imu/data_raw')
        self._imu_data = ImuYawAccumulator('/imu/data')
        self._amcl_samples = []
        self._ground_truth_samples = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._odom_subscription = self.create_subscription(
            Odometry, '/odom', self._odom_callback, qos)
        self._wheel_odom_subscription = self.create_subscription(
            Odometry, '/wheel/odom', self._wheel_odom_callback, qos)
        self._lidar_odom_subscription = self.create_subscription(
            Odometry, '/lidar/odom', self._lidar_odom_callback, qos)
        self._imu_data_raw_subscription = self.create_subscription(
            Imu, '/imu/data_raw', self._imu_data_raw_callback, qos)
        self._imu_data_subscription = self.create_subscription(
            Imu, '/imu/data', self._imu_data_callback, qos)
        self._amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_callback,
            qos,
        )
        self._ground_truth_subscription = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self._ground_truth_callback,
            qos,
        )
        self._report_timer = self.create_timer(
            report_period_sec, self.write_reports)

    def _odom_callback(self, message):
        self._odom_samples.append(_odometry_sample(message))

    def _wheel_odom_callback(self, message):
        self._wheel_odom_samples.append(_odometry_sample(message))

    def _lidar_odom_callback(self, message):
        self._lidar_odom_samples.append(_odometry_sample(message))

    def _imu_data_raw_callback(self, message):
        self._imu_data_raw.observe(message)

    def _imu_data_callback(self, message):
        self._imu_data.observe(message)

    def _amcl_callback(self, message):
        self._amcl_samples.append(PoseSample(
            stamp_ns=_stamp_ns(message.header.stamp),
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            yaw=_yaw(message.pose.pose.orientation),
            covariance=tuple(float(value) for value in message.pose.covariance),
        ))

    def _ground_truth_callback(self, message):
        self._ground_truth_samples.append(_odometry_sample(message))

    def write_reports(self):
        """Write the latest summary and matched samples atomically."""
        results = {
            'odom': evaluate_trajectory(
                self._odom_samples,
                self._ground_truth_samples,
                self._max_time_delta_ns,
                max_time_offset_ns=self._max_time_offset_ns,
                time_offset_step_ns=self._time_offset_step_ns,
            ),
            'wheel_odom': evaluate_trajectory(
                self._wheel_odom_samples,
                self._ground_truth_samples,
                self._max_time_delta_ns,
                max_time_offset_ns=self._max_time_offset_ns,
                time_offset_step_ns=self._time_offset_step_ns,
            ),
            'lidar_odom': evaluate_trajectory(
                self._lidar_odom_samples,
                self._ground_truth_samples,
                self._max_time_delta_ns,
                max_time_offset_ns=self._max_time_offset_ns,
                time_offset_step_ns=self._time_offset_step_ns,
            ),
            'imu_data_raw': evaluate_trajectory(
                self._imu_data_raw.samples,
                self._ground_truth_samples,
                self._max_time_delta_ns,
                max_time_offset_ns=self._max_time_offset_ns,
                time_offset_step_ns=self._time_offset_step_ns,
            ),
            'imu_data': evaluate_trajectory(
                self._imu_data.samples,
                self._ground_truth_samples,
                self._max_time_delta_ns,
                max_time_offset_ns=self._max_time_offset_ns,
                time_offset_step_ns=self._time_offset_step_ns,
            ),
            'amcl_pose': evaluate_trajectory(
                self._amcl_samples,
                self._ground_truth_samples,
                self._max_time_delta_ns,
                max_time_offset_ns=self._max_time_offset_ns,
                time_offset_step_ns=self._time_offset_step_ns,
            ),
        }
        for name, accumulator in (
            ('imu_data_raw', self._imu_data_raw),
            ('imu_data', self._imu_data),
        ):
            results[name].summary['valid_axes'] = ['yaw']
            results[name].summary['xy_metrics_valid'] = False
            results[name].summary['integration_method'] = (
                'trapezoidal_angular_velocity_z')
            results[name].summary['imu_diagnostics'] = (
                accumulator.diagnostics())
        summary = {
            'schema_version': 4,
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'evaluator_only_ground_truth': True,
            'passive_evaluator': True,
            'episode_id': self._episode_id,
            'arm': self._arm,
            'ground_truth_topic': '/ground_truth/odom',
            'stream_contracts': {
                'wheel_odom': 'wheel pose scale and yaw against evaluator-only ground truth',
                'lidar_odom': 'RF2O pose trajectory; never proxied by fused odom',
                'imu_data_raw': (
                    'auditable Isaac raw angular_velocity.z integration, '
                    'yaw scale/bias and source covariance'),
                'imu_data': (
                    'EKF-bound corrected angular_velocity.z integration, '
                    'yaw scale/bias and configured covariance'),
            },
            'ground_truth': {
                'input': stream_diagnostics(self._ground_truth_samples),
            },
            'estimates': {
                name: result.summary for name, result in results.items()
            },
        }
        json_path = self._output_dir / 'estimated_state_metrics.json'
        json_temporary = json_path.with_suffix('.json.tmp')
        json_temporary.write_text(
            json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)
            + '\n',
            encoding='utf-8',
        )
        json_temporary.replace(json_path)

        csv_path = self._output_dir / 'estimated_state_matches.csv'
        csv_temporary = csv_path.with_suffix('.csv.tmp')
        fieldnames = [
            'stream',
            'estimate_stamp_ns',
            'ground_truth_stamp_ns',
            'time_delta_ms',
            'estimate_x_m',
            'estimate_y_m',
            'estimate_yaw_rad',
            'aligned_x_m',
            'aligned_y_m',
            'aligned_yaw_rad',
            'ground_truth_x_m',
            'ground_truth_y_m',
            'ground_truth_yaw_rad',
            'absolute_ate_xy_m',
            'absolute_ate_yaw_rad',
            'aligned_ate_xy_m',
            'aligned_ate_yaw_rad',
            'ate_xy_m',
            'ate_yaw_rad',
            'covariance_2sigma_x_covered',
            'covariance_2sigma_y_covered',
            'covariance_2sigma_yaw_covered',
            'planar_nees',
        ]
        with csv_temporary.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for name, result in results.items():
                for row in result.rows:
                    writer.writerow({'stream': name, **row})
        csv_temporary.replace(csv_path)


def _odometry_sample(message):
    return PoseSample(
        stamp_ns=_stamp_ns(message.header.stamp),
        x=float(message.pose.pose.position.x),
        y=float(message.pose.pose.position.y),
        yaw=_yaw(message.pose.pose.orientation),
        covariance=tuple(float(value) for value in message.pose.covariance),
    )


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw(orientation):
    return math.atan2(
        2.0 * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        ),
        1.0 - 2.0 * (
            float(orientation.y) * float(orientation.y)
            + float(orientation.z) * float(orientation.z)
        ),
    )


def main(args=None):
    """Run the evaluator; finalization rewrites the latest JSON and CSV."""
    rclpy.init(args=args)
    node = None
    try:
        node = EstimatedStateEvaluator()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.write_reports()
            except Exception as error:  # Preserve shutdown while reporting I/O.
                node.get_logger().error(
                    f'Failed to write final estimated-state report: {error}')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
