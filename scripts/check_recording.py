#!/usr/bin/env python3
"""Evaluate follow_monitor CSV. This checks commands and simulation outcomes, not hardware safety."""
import argparse
import csv
import math
from pathlib import Path
import statistics
import sys

NUMERIC = (
    'wall_s', 'ros_s', 'target_x', 'target_y', 'robot_x', 'robot_y',
    'distance_m', 'bearing_deg', 'visible', 'visible_age_s', 'point_age_s',
    'cmd_v', 'cmd_w', 'cmd_age_s', 'odom_v', 'odom_w', 'odom_age_s',
)
MIN_SECONDS = dict(stationary=30, away=50, approach=60, lateral=45, turn=40,
                   occlusion=45, invalid_depth=45, demo=65, manual=10)
STOP_STATES = {'emergency_stop', 'clock_unavailable_or_paused', 'odom_unavailable',
               'target_not_reliable', 'target_stale', 'invalid_target_or_tf',
               'confirming_target', 'waiting_follow_server', 'controller_stale'}


def load_rows(path):
    with Path(path).open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        required = set(NUMERIC) | {'state'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('CSV columns missing; record with follow_monitor.')
        rows = []
        for line, row in enumerate(reader, 2):
            try:
                values = {key: float(row[key]) for key in NUMERIC}
                if not all(math.isfinite(value) for value in values.values()):
                    raise ValueError('non-finite value')
                if values['visible'] not in (0, 1) or not row['state']:
                    raise ValueError('invalid flag/state')
                values['state'] = row['state']
                rows.append(values)
            except (TypeError, KeyError, ValueError) as error:
                raise ValueError(f'Invalid CSV line {line}: {error}') from error
    if len(rows) < 20:
        raise ValueError('Recording too short or no odometry/target truth received.')
    return rows


def evaluate(rows, scenario='manual', desired_distance=1.5):
    failures = []
    notes = []
    first, last = rows[0], rows[-1]
    elapsed = last['ros_s'] - first['ros_s']
    if elapsed < MIN_SECONDS[scenario]:
        failures.append(f'{scenario} needs >= {MIN_SECONDS[scenario]} simulation seconds; got {elapsed:.1f}')
    if any(b['wall_s'] <= a['wall_s'] or b['ros_s'] < a['ros_s']
           for a, b in zip(rows, rows[1:])):
        failures.append('Time reversed or wall timestamps repeated: record a new run after resetting Gazebo.')
    warm = [row for row in rows if row['wall_s'] - first['wall_s'] >= 1]
    if not warm:
        failures.append('No samples beyond startup.')
        return failures, notes
    if any(row['cmd_v'] < -0.001 for row in rows):
        failures.append('Reverse command detected.')
    if any(row['cmd_v'] > 0.201 or abs(row['cmd_w']) > 0.601 for row in rows):
        failures.append('Command exceeded default 0.20 m/s or 0.60 rad/s limit.')
    if any(row['cmd_age_s'] > 0.25 for row in warm):
        failures.append('/cmd_vel missing/stale (>0.25 wall seconds).')
    # Topics arrive separately. Give command/state/flag subscribers 0.12 s to agree.
    invalid_since = None
    state_since = rows[0]['wall_s']
    previous_state = rows[0]['state']
    bad_invalid = bad_state = bad_forward = False
    losses = []
    for row in rows:
        wall = row['wall_s']
        invalid = (row['visible'] == 0 or row['visible_age_s'] > 0.5 or
                   row['point_age_s'] > 0.5 or row['odom_age_s'] > 0.5)
        if invalid:
            if invalid_since is None:
                invalid_since = wall
            if wall - invalid_since > 0.12 and (abs(row['cmd_v']) > 0.001 or abs(row['cmd_w']) > 0.001):
                bad_invalid = True
        elif invalid_since is not None:
            if wall - invalid_since >= 0.5:
                losses.append((invalid_since, wall))
            invalid_since = None
        if row['state'] != previous_state:
            previous_state, state_since = row['state'], wall
        if wall - state_since > 0.12:
            if row['state'] in STOP_STATES and (abs(row['cmd_v']) > 0.001 or abs(row['cmd_w']) > 0.001):
                bad_state = True
            if (row['state'].startswith('hold_distance') or row['state'] == 'align_target') and abs(row['cmd_v']) > 0.001:
                bad_forward = True
    if bad_invalid:
        failures.append('Nonzero command after invalid/stale input exceeded timeout plus subscriber grace.')
    if bad_state:
        failures.append('STOP state continued to produce a nonzero command.')
    if bad_forward:
        failures.append('Forward command while holding distance or turning in place.')
    moved = max(row['cmd_v'] for row in rows) > 0.02
    if scenario != 'manual' and not moved:
        failures.append('No forward following observed; zero commands alone do not pass the scenario.')
    if scenario in ('occlusion', 'invalid_depth', 'demo'):
        required_losses = 3 if scenario != 'demo' else 2
        recovered = [end for _, end in losses if any(
            row['wall_s'] > end and row['wall_s'] < end + 5 and
            row['state'] == 'following' and row['cmd_v'] > 0.005 for row in rows)]
        if len(recovered) < required_losses:
            failures.append(f'Need {required_losses} completed loss-and-resume events; observed {len(recovered)}.')
    if scenario in ('stationary', 'away', 'turn', 'occlusion', 'invalid_depth'):
        tail = [row for row in rows if row['ros_s'] >= last['ros_s'] - 8 and
                row['visible'] and row['odom_age_s'] < 0.5]
        if len(tail) < 20:
            failures.append('Not enough reliable samples in final 8 simulation seconds.')
        else:
            error = statistics.median(abs(row['distance_m'] - desired_distance) for row in tail)
            notes.append(f'Final 8 s median absolute distance error: {error:.3f} m (limit 0.15 m)')
            if error > 0.15:
                failures.append('Final following distance did not converge within 0.15 m.')
    if scenario == 'approach':
        close = [row for row in rows if row['distance_m'] < desired_distance - 0.05]
        if not close:
            failures.append('Approach scenario never reached the close-distance test interval.')
        elif any(row['cmd_v'] > 0.001 for row in close):
            failures.append('Robot kept advancing when target was closer than desired distance.')
        hold = next((row['wall_s'] for row in close), None)
        if hold is not None and not any(row['wall_s'] > hold + 1 and row['cmd_v'] > 0.02 for row in rows):
            failures.append('No resumed following after the target moved away.')
    if scenario in ('lateral', 'turn') and not any(abs(row['cmd_w']) > 0.03 for row in rows):
        failures.append('No steering observed in lateral/turn scenario.')
    notes.extend([
        f'Simulation duration: {elapsed:.1f} s; rows: {len(rows)}',
        f'Max commands: {max(row["cmd_v"] for row in rows):.3f} m/s, '
        f'{max(abs(row["cmd_w"]) for row in rows):.3f} rad/s',
        f'Completed invalid-input intervals >=0.5 s: {len(losses)}',
    ])
    return failures, notes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path')
    parser.add_argument('--scenario', choices=MIN_SECONDS, default='manual')
    parser.add_argument('--desired-distance', type=float, default=1.5)
    args = parser.parse_args()
    if not math.isfinite(args.desired_distance) or args.desired_distance <= 0.3:
        parser.error('--desired-distance must be finite and >0.3 m')
    try:
        failures, notes = evaluate(load_rows(args.csv_path), args.scenario, args.desired_distance)
    except (OSError, ValueError) as error:
        print(f'INVALID RECORDING: {error}', file=sys.stderr)
        return 2
    for note in notes:
        print(note)
    for failure in failures:
        print(f'FAIL: {failure}')
    print('PASS: recorded checks only; review RViz and physical odometry as well.' if not failures else 'FAIL')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
