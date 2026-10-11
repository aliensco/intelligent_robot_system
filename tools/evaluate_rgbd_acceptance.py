#!/usr/bin/env python
"""Offline RGB-D field evidence adapter. Reads JSONL or ROS bags, never sends commands."""
from __future__ import division, print_function
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate_rgbd_trials import percentile

SCENARIOS = ('straight_follow', 'obstacle_left', 'obstacle_right',
             'multiple_obstacles', 'occlusion', 'crossing')
OBSTACLE_SCENARIOS = ('obstacle_left', 'obstacle_right', 'multiple_obstacles', 'crossing')
VERDICTS = ('scenario_exercised', 'collision_free', 'correct_identity', 'no_intervention')


def finite(value):
    return isinstance(value, (int, float)) and not (math.isnan(value) or math.isinf(value))


def fresh(stamp, now, timeout):
    return finite(stamp) and 0 <= now-stamp <= timeout


def summarize(rows, trial):
    """Missing evidence is INCOMPLETE; observed failed criteria are FAIL."""
    result = dict(status='INCOMPLETE', failures=[], missing=[], metrics={})
    fail, missing, metrics = result['failures'], result['missing'], result['metrics']
    rows = [r for r in rows if r.get('kind') == 'sample']
    if len(rows) < 2:
        missing.append('insufficient_samples')
        return result
    cfg = dict(timeout_s=.5, max_sample_gap_s=.5, min_duration_s=5.,
               min_travel_m=.5, min_coverage=.95, follow_distance_m=.6,
               minimum_distance_m=.4, gap_error_p95_limit_m=.6,
               max_target_loss_s=3., stopped_linear_m_s=.015,
               stopped_angular_rad_s=.03, stopped_duration_s=.5)
    cfg.update(trial.get('criteria', {}))
    times = [r['t'] for r in rows]
    deltas = [b-a for a, b in zip(times, times[1:])]
    if any(not finite(v) for v in times):
        missing.append('nonfinite_timestamps')
        return result
    numeric_fields = ('speed', 'angular_speed', 'target_distance_m', 'costmap_clearance_m')
    if any((r.get(name) is not None and not finite(r[name])) for r in rows for name in numeric_fields):
        missing.append('nonfinite_measurements')
        return result
    if any(r.get('xy') is not None and (len(r['xy']) != 2 or not all(finite(v) for v in r['xy'])) for r in rows):
        missing.append('invalid_odometry_position')
        return result
    if any(d <= 0 or d > cfg['max_sample_gap_s'] for d in deltas):
        missing.append('sampling_gap_or_clock_reset')
        return result
    duration = times[-1]-times[0]
    if duration <= 0:
        missing.append('nonpositive_duration')
        return result
    coverage = dict((name, 0.) for name in ('odom', 'rgb', 'depth', 'grid', 'sensor'))
    loss_start, longest_loss, visible_time = None, 0., 0.
    losses, errors, clearances, distances = [], [], [], []
    travel, previous_xy, end_stopped_since = 0., None, None
    for i, row in enumerate(rows):
        t = row['t']
        dt = deltas[i] if i < len(deltas) and deltas[i] > 0 else 0.
        valid = {}
        for name in coverage:
            valid[name] = fresh(row.get(name+'_stamp'), t, cfg['timeout_s'])
            coverage[name] += dt if valid[name] else 0.
        tracked = (row.get('target_valid') is True and
                   fresh(row.get('target_stamp'), t, cfg['timeout_s']) and
                   fresh(row.get('target_valid_stamp'), t, cfg['timeout_s']))
        visible_time += dt if tracked else 0.
        if not tracked and loss_start is None:
            loss_start = t
        if tracked and loss_start is not None:
            losses.append(t-loss_start)
            loss_start = None
        if valid['odom'] and row.get('xy') is not None:
            if previous_xy is not None:
                travel += math.hypot(row['xy'][0]-previous_xy[0], row['xy'][1]-previous_xy[1])
            previous_xy = row['xy']
            if tracked and row.get('target_distance_m') is not None:
                distance = row['target_distance_m']
                distances.append(distance)
                errors.append(abs(distance-cfg['follow_distance_m']))
            if (row.get('speed') is not None and abs(row['speed']) <= cfg['stopped_linear_m_s'] and
                    row.get('angular_speed') is not None and abs(row['angular_speed']) <= cfg['stopped_angular_rad_s']):
                if end_stopped_since is None:
                    end_stopped_since = t
            else:
                end_stopped_since = None
        else:
            previous_xy, end_stopped_since = None, None
        if valid['grid'] and valid['odom'] and row.get('costmap_clearance_m') is not None:
            clearances.append(row['costmap_clearance_m'])
        if valid['sensor'] and any(row.get(k, 0) for k in ('bumper', 'cliff', 'wheel_drop')):
            fail.append('base_contact_or_drop_event')
    if loss_start is not None:
        losses.append(times[-1]-loss_start)
    longest_loss = max(losses) if losses else 0.
    coverage = dict((name, value/duration) for name, value in coverage.items())
    metrics.update(duration_s=duration, travel_m=travel, sensor_coverage=coverage,
                   visible_fraction=visible_time/duration, longest_target_loss_s=longest_loss,
                   target_loss_intervals_s=losses, gap_error_p95_m=percentile(errors, .95),
                   minimum_target_distance_m=min(distances) if distances else None,
                   minimum_costmap_clearance_m=min(clearances) if clearances else None,
                   terminal_reason=trial.get('terminal_reason'))
    for name, value in coverage.items():
        if value < cfg['min_coverage']:
            missing.append('insufficient_fresh_'+name)
    if duration < cfg['min_duration_s'] or travel < cfg['min_travel_m']:
        missing.append('insufficient_duration_or_travel')
    if not errors:
        missing.append('no_valid_follow_distance')
    elif metrics['gap_error_p95_m'] > cfg['gap_error_p95_limit_m']:
        fail.append('follow_distance_error')
    if distances and min(distances) < cfg['minimum_distance_m']:
        fail.append('minimum_target_distance_breached')
    if longest_loss > cfg['max_target_loss_s']:
        fail.append('target_loss_over_limit')
    if clearances and min(clearances) < 0:
        fail.append('costmap_footprint_overlap')
    if trial['scenario'] in OBSTACLE_SCENARIOS and not clearances:
        missing.append('no_measured_obstacle_clearance')
    if trial['scenario'] == 'multiple_obstacles' and trial.get('observed_obstacle_count', 0) < 2:
        missing.append('multiple_obstacles_not_documented')
    verdict = trial.get('verdict', {})
    for name in VERDICTS:
        if verdict.get(name) is False:
            fail.append('human_verdict:'+name)
        elif verdict.get(name) is not True:
            missing.append('human_verdict:'+name)
    endpoint = trial.get('endpoint')
    if endpoint is None:
        missing.append('expected_endpoint_not_defined')
    elif previous_xy is None:
        missing.append('final_pose_unavailable')
    else:
        error = math.hypot(previous_xy[0]-endpoint['xy'][0], previous_xy[1]-endpoint['xy'][1])
        metrics['endpoint_error_m'] = error
        if error > endpoint.get('tolerance_m', .15):
            missing.append('expected_endpoint_not_reached')
    if trial.get('terminal_reason') not in ('finished', 'goal_reached', 'operator_completed'):
        missing.append('not_a_completed_run')
    if verdict.get('route_completed') is not True:
        missing.append('route_completion_not_confirmed')
    if end_stopped_since is None or times[-1]-end_stopped_since < cfg['stopped_duration_s']:
        missing.append('no_measured_stopped_finish')
    result['failures'], result['missing'] = sorted(set(fail)), sorted(set(missing))
    result['status'] = 'FAIL' if fail else ('INCOMPLETE' if missing else 'PASS')
    return result


def _stamp(msg):
    return msg.header.stamp.to_sec() if hasattr(msg, 'header') else None


def _clearance(grid, xy, radius):
    origin, resolution = grid.info.origin, grid.info.resolution
    q = origin.orientation
    angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    c, s = math.cos(angle), math.sin(angle)
    distances = []
    for i, value in enumerate(grid.data):
        if value != 100:
            continue
        x, y = (i % grid.info.width+.5)*resolution, (i // grid.info.width+.5)*resolution
        distances.append(math.hypot(origin.position.x+c*x-s*y-xy[0],
                                    origin.position.y+s*x+c*y-xy[1])-radius-resolution/math.sqrt(2))
    return min(distances) if distances else None


def bag_samples(path, trial):
    """ROS import is lazy: JSONL evaluation needs only the Python standard library."""
    import rosbag
    ns = trial.get('namespace', '/method3_motion').rstrip('/')
    topics = dict(odom='/odom', sensor='/mobile_base/sensors/core',
                  rgb='/camera/rgb/image_rect_color',
                  depth='/camera/depth_registered/hw_registered/image_rect',
                  grid=ns+'/move_base/local_costmap/costmap', target=ns+'/target_pose',
                  target_valid=ns+'/target_valid', perception=ns+'/perception_status')
    topics.update(trial.get('topics', {}))
    inverse, latest = dict((value, key) for key, value in topics.items()), {}
    rows, last_sample = [], None
    with rosbag.Bag(path) as bag:
        for topic, msg, received in bag.read_messages(topics=list(inverse)):
            t = received.to_sec()
            name = inverse[topic]
            latest[name] = (msg, t)
            if t < trial.get('start_time', -float('inf')):
                continue
            if t > trial.get('end_time', float('inf')):
                break
            contact = name == 'sensor' and any(getattr(msg, f, 0) for f in ('bumper', 'cliff', 'wheel_drop'))
            if last_sample is not None and t-last_sample < .099 and not contact:
                continue
            last_sample = t
            row = dict(kind='sample', t=t, target_valid=False)
            for name in ('odom', 'sensor', 'rgb', 'depth', 'grid'):
                if name in latest:
                    current, receipt = latest[name]
                    row[name+'_stamp'] = _stamp(current)
                    # SensorState has a header. Never replace a zero/headerless
                    # source stamp with a fresh receipt to make stalled data pass.
            if 'perception' in latest:
                try:
                    evidence = json.loads(latest['perception'][0].data)
                    if not isinstance(evidence, dict):
                        evidence = {}
                except (ValueError, TypeError):
                    evidence = {}
                # The node records these source stamps only after check_pair.
                # A watchdog rejection repeats old stamps; fresh status receipt
                # alone must not make an unplugged camera appear current.
                for name in ('rgb', 'depth'):
                    if name not in latest and finite(evidence.get(name+'_stamp')):
                        row[name+'_stamp'] = evidence[name+'_stamp']
            if 'target_valid' in latest:
                row['target_valid'] = bool(latest['target_valid'][0].data)
                row['target_valid_stamp'] = latest['target_valid'][1]
            if 'sensor' in latest:
                current = latest['sensor'][0]
                for field in ('bumper', 'cliff', 'wheel_drop'):
                    row[field] = getattr(current, field)
            if 'odom' in latest:
                odom = latest['odom'][0]
                position, speed = odom.pose.pose.position, odom.twist.twist
                row.update(xy=[position.x, position.y], speed=speed.linear.x,
                           angular_speed=speed.angular.z)
                frame = odom.header.frame_id.lstrip('/')
                if 'target' in latest:
                    target = latest['target'][0]
                    if target.header.frame_id.lstrip('/') == frame:
                        row['target_stamp'] = _stamp(target)
                        row['target_distance_m'] = math.hypot(target.pose.position.x-position.x,
                                                             target.pose.position.y-position.y)
                if 'grid' in latest:
                    grid = latest['grid'][0]
                    if grid.header.frame_id.lstrip('/') == frame:
                        row['costmap_clearance_m'] = _clearance(grid, row['xy'], trial.get('footprint_radius_m', .26))
                    else:
                        row['grid_stamp'] = None
            rows.append(row)
    return rows


def evaluate(manifest, directory):
    required = manifest.get('required_scenarios', list(SCENARIOS))
    repeats = manifest.get('repeats', 3)
    if not required or repeats < 1:
        raise ValueError('At least one required scenario and one repetition are needed')
    results, seen = [], set()
    streak = dict((name, 0) for name in required)
    states = dict((name, 'INCOMPLETE') for name in required)
    for trial in manifest.get('trials', []):
        trial = dict(trial)
        path = os.path.realpath(os.path.join(directory, trial['log']))
        if path in seen:
            raise ValueError('A recording cannot count as multiple independent trials')
        seen.add(path)
        if trial.get('motion_log'):
            with open(os.path.join(directory, trial['motion_log'])) as source:
                summaries = []
                for line in source:
                    try:
                        item = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(item, dict) and 'final_reason' in item:
                        summaries.append(item)
                trial['terminal_reason'] = summaries[-1]['final_reason'] if summaries else None
        if path.endswith('.bag'):
            rows = bag_samples(path, trial)
        else:
            with open(path) as source:
                rows = [json.loads(line) for line in source if line.strip()]
        result = summarize(rows, trial)
        result.update(scenario=trial['scenario'], log=trial['log'])
        results.append(result)
        name = trial['scenario']
        if name in streak:
            streak[name] = streak[name]+1 if result['status'] == 'PASS' else 0
            states[name] = ('PASS' if streak[name] >= repeats else
                            ('FAIL' if result['status'] == 'FAIL' else 'INCOMPLETE'))
    overall = ('PASS' if all(v == 'PASS' for v in states.values()) else
               ('FAIL' if 'FAIL' in states.values() else 'INCOMPLETE'))
    return dict(status=overall, scenarios=states, ending_pass_streaks=streak,
                repeats_required=repeats, trials=results,
                note='Engineering evidence only; map clearance is not physical collision ground truth. No competition score is inferred.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', help='JSON scenario manifest; paths relative to it')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    with open(args.manifest) as source:
        manifest = json.load(source)
    report = evaluate(manifest, os.path.dirname(os.path.abspath(args.manifest)))
    with open(args.output, 'w') as destination:
        json.dump(report, destination, indent=2, allow_nan=False)
    print(json.dumps(dict(status=report['status'], scenarios=report['scenarios'])))
    return {'PASS': 0, 'FAIL': 1, 'INCOMPLETE': 2}[report['status']]


if __name__ == '__main__':
    sys.exit(main())
