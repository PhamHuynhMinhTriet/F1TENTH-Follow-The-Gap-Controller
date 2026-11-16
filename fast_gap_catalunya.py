#!/usr/bin/env python3
"""
F1TENTH reactive controller for the Catalunya map.
ROS 2 Humble | Follow-the-Gap + Disparity Extender

Topics:
  Subscribes: /scan, /ego_racecar/odom
  Publishes:  /drive (AckermannDriveStamped)

IMPORTANT:
  vmax=vmin=50.0 is intentionally retained as requested. With the
  supplied speed formula, vmin=vmax can prevent obstacle braking from
  reducing the commanded speed. Check simulator max_speed and test at
  low speed before attempting a run.
"""

import csv
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class FastGapCore:
    def __init__(self, p, log=print):
        self.p = p
        self.log = log
        self.vmax = float(p['vmax'])
        self.vmin = float(p['vmin'])
        self.decel = float(p['decel'])
        self.dmargin = float(p['dmargin'])
        self.half_width = float(p['half_width'])
        self.disp_thresh = float(p['disp_thresh'])
        self.max_range = float(p['max_range'])
        self.fov = math.radians(float(p['fov_deg']))
        self.kp = float(p['kp'])
        self.sref = float(p['sref'])
        self.alpha = float(p['alpha'])
        self.alpha_straight = float(p['alpha_straight'])
        self.steer_db = float(p['steer_db'])
        self.kc = float(p['kc'])
        self.cdist = float(p['cdist'])
        self.center_max = float(p['center_max'])
        self.center_alpha = float(p['center_alpha'])
        self.side_lo = math.radians(float(p['side_lo_deg']))
        self.vstep = float(p['vstep'])
        self.vmax_max = float(p['vmax_max'])
        self.clear_thr = float(p['clear_thr'])
        self.autotune = bool(p['autotune'])
        self.freeze = bool(p['freeze'])
        self.lap_leave = float(p['lap_leave'])
        self.lap_return = float(p['lap_return'])
        self.zone_radius = float(p['zone_radius'])
        self.state_file = str(p['state_file'])
        self.log_file = str(p['log_file'])

        self.max_steer = float(p.get('max_steer', 0.4189))
        self.prev_steer = 0.0
        self.start_xy = None
        self.left_start = False
        self.lap_start_time = None
        self.lap_number = 0
        self.min_clearance = float('inf')
        self.best_vmax = self.vmax
        self.dirty_zones = []
        self.scan_count = 0
        self.last_log_time = 0.0

        if bool(p['load']):
            self.load()

        self._ensure_csv()

    def _ensure_csv(self):
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['lap', 'lap_time_s', 'minimum_side_clearance_m',
                     'vmax_mps', 'clean']
                )

    def load(self):
        try:
            with open(self.state_file, 'r') as f:
                state = json.load(f)
            self.vmax = float(state.get('vmax', self.vmax))
            self.best_vmax = float(state.get('best_vmax', self.vmax))
            self.dirty_zones = state.get('dirty_zones', [])
            self.log('Loaded controller state from ' + self.state_file)
        except (OSError, ValueError, TypeError):
            self.log('No valid saved state found; using configured defaults.')

    def save(self):
        state = {
            'vmax': self.vmax,
            'best_vmax': self.best_vmax,
            'dirty_zones': self.dirty_zones,
        }
        tmp = self.state_file + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(state, f, indent=2)
        os.replace(tmp, self.state_file)

    @staticmethod
    def _sanitize(ranges, max_range):
        scan = np.asarray(ranges, dtype=np.float32)
        scan = np.nan_to_num(scan, nan=max_range, posinf=max_range,
                             neginf=0.0)
        return np.clip(scan, 0.0, max_range)

    def _preprocess(self, ranges, angle_min, angle_increment):
        scan = self._sanitize(ranges, self.max_range)
        angles = angle_min + np.arange(len(scan)) * angle_increment
        mask = np.abs(angles) <= self.fov
        if not np.any(mask):
            return scan, angles
        return scan[mask], angles[mask]

    def _extend_disparities(self, ranges):
        """Extend the nearer side of each large adjacent-beam disparity."""
        out = ranges.copy()
        if len(out) < 2:
            return out
        for i in np.where(np.abs(np.diff(ranges)) > self.disp_thresh)[0]:
            near_idx = i if ranges[i] < ranges[i + 1] else i + 1
            distance = max(float(ranges[near_idx]), 0.05)
            # Approximate angular width subtended by half_width.
            beam_angle = 2.0 * math.atan2(self.half_width, distance)
            # Number of beams is calculated later from angle increment.
            self._last_extension = (near_idx, beam_angle)
            if ranges[i] < ranges[i + 1]:
                out[i + 1] = min(out[i + 1], ranges[i])
            else:
                out[i] = min(out[i], ranges[i + 1])
        return out

    def _extend_with_angle(self, ranges, angle_increment):
        out = ranges.copy()
        if len(out) < 2 or angle_increment <= 0:
            return out
        disparities = np.where(np.abs(np.diff(ranges)) > self.disp_thresh)[0]
        for i in disparities:
            near_idx = i if ranges[i] < ranges[i + 1] else i + 1
            distance = max(float(ranges[near_idx]), 0.05)
            half_angle = math.atan2(self.half_width, distance)
            count = max(1, int(math.ceil(half_angle / angle_increment)))
            lo = max(0, near_idx - count)
            hi = min(len(out), near_idx + count + 1)
            out[lo:hi] = np.minimum(out[lo:hi], ranges[near_idx])
        return out

    def _smooth(self, ranges):
        if len(ranges) < 5:
            return ranges
        kernel = np.array([1, 2, 3, 2, 1], dtype=np.float32) / 9.0
        return np.convolve(ranges, kernel, mode='same')

    def _find_target(self, ranges, angles):
        if len(ranges) == 0:
            return 0.0
        # Choose the most open contiguous region; among near-maximum
        # beams choose the one closest to straight ahead.
        threshold = 0.90 * float(np.max(ranges))
        candidates = np.where(ranges >= threshold)[0]
        if len(candidates) == 0:
            idx = int(np.argmax(ranges))
        else:
            idx = int(candidates[np.argmin(np.abs(angles[candidates]))])
        return float(angles[idx])

    def lateral_sides(self, ranges, angles):
        left_mask = (angles >= self.side_lo) & (angles <= self.fov)
        right_mask = (angles <= -self.side_lo) & (angles >= -self.fov)
        left = float(np.min(ranges[left_mask])) if np.any(left_mask) else self.max_range
        right = float(np.min(ranges[right_mask])) if np.any(right_mask) else self.max_range
        return left, right

    def side_clearance(self, ranges, angles):
        left, right = self.lateral_sides(ranges, angles)
        return min(left, right)

    def zone_factor(self, x, y):
        factor = 1.0
        for zone in self.dirty_zones:
            zx, zy = float(zone['x']), float(zone['y'])
            distance = math.hypot(x - zx, y - zy)
            radius = max(float(zone.get('radius', self.zone_radius)), 0.1)
            if distance < radius:
                # Local speed multiplier: 0.4 at zone centre, rising to 1.
                factor = min(factor, 0.4 + 0.6 * distance / radius)
        return factor

    def lap_done(self, x, y, now):
        if self.start_xy is None:
            self.start_xy = (x, y)
            self.lap_start_time = now
            return
        distance = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
        if distance > self.lap_leave:
            self.left_start = True
        if self.left_start and distance <= self.lap_return:
            elapsed = max(0.0, now - self.lap_start_time)
            self.lap_number += 1
            clean = self.min_clearance >= self.clear_thr
            self.log(
                f'Lap {self.lap_number}: {elapsed:.3f} s | '
                f'min side clearance={self.min_clearance:.3f} m | '
                f'{"CLEAN" if clean else "CONTACT/UNCLEAN"}'
            )
            with open(self.log_file, 'a', newline='') as f:
                csv.writer(f).writerow([
                    self.lap_number, f'{elapsed:.3f}',
                    f'{self.min_clearance:.3f}', f'{self.vmax:.2f}', clean
                ])

            if clean:
                self.best_vmax = max(self.best_vmax, self.vmax)
                if self.autotune and not self.freeze:
                    self.vmax = min(self.vmax + self.vstep, self.vmax_max)
            else:
                # Record a local slow zone around the point where the
                # smallest measured side clearance occurred.
                if self.last_xy is not None:
                    self.dirty_zones.append({
                        'x': self.last_xy[0], 'y': self.last_xy[1],
                        'radius': self.zone_radius
                    })
                if self.autotune and not self.freeze:
                    self.vmax = max(self.vmin, min(self.vmax, self.best_vmax))
            self.save()
            self.left_start = False
            self.lap_start_time = now
            self.min_clearance = float('inf')
            self.start_xy = (x, y)

    def step(self, ranges, angle_min, angle_increment, x, y, now):
        self.last_xy = (x, y)
        scan, angles = self._preprocess(ranges, angle_min, angle_increment)
        if len(scan) == 0:
            return 0.0, self.vmin

        extended = self._extend_with_angle(scan, angle_increment)
        smooth = self._smooth(extended)
        target = self._find_target(smooth, angles)

        steer_raw = self.kp * target
        if abs(steer_raw) < self.steer_db:
            steer_raw = 0.0

        left, right = self.lateral_sides(scan, angles)
        center = 0.0
        if min(left, right) < self.cdist:
            center = float(np.clip(self.kc * (left - right),
                                   -self.center_max, self.center_max))
        steer_raw = float(np.clip(steer_raw + center,
                                  -self.max_steer, self.max_steer))

        alpha = self.alpha_straight if abs(steer_raw) < self.sref else self.alpha
        steer = alpha * self.prev_steer + (1.0 - alpha) * steer_raw
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        self.prev_steer = steer

        front_mask = np.abs(angles) <= math.radians(6.0)
        d_front = float(np.min(scan[front_mask])) if np.any(front_mask) else float(np.min(scan))
        clearance = self.side_clearance(scan, angles)
        self.min_clearance = min(self.min_clearance, clearance)

        vlim = self.vmax * self.zone_factor(x, y)
        v_curve = vlim - (vlim - min(self.vmin, vlim)) * min(
            1.0, abs(steer) / self.sref
        )
        v_brake = math.sqrt(
            2.0 * self.decel * max(d_front - self.dmargin, 0.0)
        )
        # Retains the exact speed-clamping structure supplied by the user.
        v = max(min(self.vmin, vlim), min(v_curve, v_brake, vlim))

        self.lap_done(x, y, now)
        return steer, float(v)


class FastGap(Node):
    def __init__(self):
        super().__init__('fast_gap_controller')
        defaults = dict(
            vmax=50.0, vmin=50.0, decel=7.0, dmargin=1.0, half_width=0.25,
            disp_thresh=0.5, max_range=10.0, fov_deg=90.0, kp=1.0, sref=0.55,
            alpha=0.9, alpha_straight=0.4, steer_db=0.03,
            kc=0.4, cdist=1.0, center_max=0.15, center_alpha=1.0,
            side_lo_deg=35.0, vstep=0.25, vmax_max=50.0,
            clear_thr=0.17, autotune=True, load=False, freeze=False, skip=2,
            lap_leave=40.0, lap_return=3.0, zone_radius=4.0,
            state_file='catalunya_state.json', log_file='catalunya_laps.csv',
            max_steer=0.4189
        )
        p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.skip = max(1, int(p['skip']))
        self.cnt = 0
        self.pos = None
        self.core = FastGapCore(p, log=self.get_logger().info)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.get_logger().info('FastGap Catalunya controller started.')

    def odom_cb(self, msg):
        self.pos = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y)
        )

    def scan_cb(self, msg):
        self.cnt += 1
        if self.pos is None or self.cnt % self.skip:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        steer, speed = self.core.step(
            msg.ranges, msg.angle_min, msg.angle_increment,
            self.pos[0], self.pos[1], now
        )
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.steering_angle = float(steer)
        out.drive.speed = float(speed)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FastGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
