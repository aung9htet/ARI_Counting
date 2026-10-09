"""ROS-independent trajectory and timestamp validation."""
import math
import numpy as np


def validate_targets(targets, limits):
    if len(targets) != len(limits):
        raise ValueError(f'expected {len(limits)} joint targets')
    for target, limit in zip(targets, limits):
        if not math.isfinite(target) or not limit['lower'] <= target <= limit['upper']:
            raise ValueError(f"{limit['name']} target outside [{limit['lower']:.4f}, {limit['upper']:.4f}] rad")


def trajectory_samples(start, waypoints, limits):
    previous, previous_time = list(start), 0.0
    samples = []
    for point in waypoints:
        targets, duration = point['positions'], float(point['time'])
        validate_targets(targets, limits)
        if not math.isfinite(duration) or duration <= previous_time:
            raise ValueError('waypoint times must be positive, finite and strictly increasing')
        dt = duration-previous_time
        # Quintic interpolation peaks at 1.875 times mean velocity.
        if any(1.875*abs(b-a)/dt > limit['velocity']*0.5 for a,b,limit in zip(previous,targets,limits)):
            raise ValueError('trajectory exceeds 50% of URDF joint velocity limits; increase duration')
        count = max(2, math.ceil(max(abs(b-a) for a,b in zip(previous,targets))/0.02), math.ceil(dt/0.1))
        for index in range(count+1):
            u=index/count
            smooth=10*u**3-15*u**4+6*u**5
            samples.append([a+(b-a)*smooth for a,b in zip(previous,targets)])
        previous, previous_time = list(targets), duration
    if not waypoints:
        raise ValueError('at least one waypoint is required')
    return samples


def past_indices(source_ns, target_ns, max_age_ns=100_000_000):
    source = np.asarray(source_ns, dtype=np.int64)
    targets = np.asarray(target_ns, dtype=np.int64)
    if len(source) and np.any(np.diff(source) < 0):
        raise ValueError('simulation timestamps moved backwards; split into separate episodes')
    indices = np.searchsorted(source, targets, side='right') - 1
    safe = np.maximum(indices, 0)
    valid = indices >= 0
    if len(source):
        valid &= targets-source[safe] <= max_age_ns
    else:
        valid[:] = False
    return np.where(valid, indices, -1), valid
