"""Identity association in a fixed odometry frame. Python 2.7 compatible."""
import math
from .depth import finite


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Tracker(object):
    def __init__(self, confirmations=4, timeout=0.5, speed=0.5, jitter=0.12,
                 reacquire_radius=0.45, ambiguity_margin=0.10,
                 identity_reacquire_radius=1.5, reacquire_confirmations=6, empty_grace=0.):
        if (not isinstance(confirmations, int) or confirmations < 2 or
                not all(finite(v) and v > 0 for v in
                        (timeout, speed, jitter, reacquire_radius, ambiguity_margin, identity_reacquire_radius)) or
                not isinstance(reacquire_confirmations, int) or reacquire_confirmations < confirmations or
                identity_reacquire_radius < reacquire_radius):
            raise ValueError('Invalid tracker configuration')
        self.confirmations, self.timeout = confirmations, timeout
        if not finite(empty_grace) or not 0 <= empty_grace <= min(.15, timeout):
            raise ValueError('Empty-observation grace must be within 0..min(.15, timeout)')
        self.empty_grace = empty_grace
        self.speed, self.jitter = speed, jitter
        self.reacquire_radius, self.margin = reacquire_radius, ambiguity_margin
        self.anchor = None
        self.stamp = self.input_stamp = None
        self.count = 0
        self.locked = False
        self.reason = 'acquiring'
        self.identity_radius = identity_reacquire_radius
        self.reacquire_confirmations = reacquire_confirmations
        self.pending = self.pending_stamp = None
        self.pending_count = 0
        self.pending_strong = None
        self.reacquired = False

    def miss(self, reason='target_lost'):
        self.count = 0
        self.pending = self.pending_stamp = None
        self.pending_count = 0
        self.pending_strong = None
        self.reason = reason
        return None

    def update(self, candidates, stamp, verified=None):
        self.reacquired = False
        if not finite(stamp) or stamp <= 0 or (self.input_stamp is not None and stamp <= self.input_stamp):
            return self.miss('out_of_order_target')
        self.input_stamp = stamp
        verified = verified if verified is not None else [False] * len(candidates)
        if len(verified) != len(candidates):
            return self.miss('identity_metadata_mismatch')
        pairs = [(p, v is True) for p, v in zip(candidates, verified)
                 if len(p) == 3 and all(finite(x) for x in p)]
        candidates = [p for p, v in pairs]
        if (not candidates and self.locked and self.count > 0 and self.stamp is not None and
                0 <= stamp-self.stamp <= self.empty_grace):
            # Missing depth for one frame still reports NO target observation.
            # Preserve association only, so the next actual compatible sample
            # need not rebuild a four-frame confirmation. Do not renew anchor
            # timestamps, return an invented pose, or relax ambiguity handling.
            self.reason = 'observation_gap'
            return None
        if self.stamp is not None and stamp - self.stamp > self.timeout:
            self.count = 0
            if not self.locked:
                self.anchor = None
        # After a miss, the last confirmed anchor is immutable until multiple
        # fresh, appearance-verified observations agree. Two plausible objects
        # remain ambiguous, even when one happens to be nearer.
        if self.locked and self.count == 0:
            # An impossible remote candidate must not switch the verification
            # mode of an otherwise recoverable nearby candidate. Filter using
            # the widest physically reachable range before considering strength.
            reachable = min(self.identity_radius,
                            self.jitter + self.speed * (stamp - self.stamp))
            pairs = [(p, v) for p, v in pairs if distance(p, self.anchor) <= reachable]
            strong = any(v for p, v in pairs)
            gate = reachable if strong else min(self.reacquire_radius, reachable)
            # Keep weak competitors inside a widened search region: appearance
            # strength alone cannot resolve two physically plausible identities.
            eligible = [(p, v) for p, v in pairs if distance(p, self.anchor) <= gate]
            if len(eligible) != 1:
                return self.miss('ambiguous_target' if len(eligible) > 1 else 'target_lost')
            selected, strong = eligible[0]
            if (self.pending is None or self.pending_strong != strong or
                    stamp-self.pending_stamp > self.timeout or
                    distance(selected, self.pending) > self.jitter+self.speed*(stamp-self.pending_stamp)):
                self.pending_count = 0
            self.pending, self.pending_stamp = selected, stamp
            self.pending_strong = strong
            self.pending_count += 1
            self.reason = 'reconfirming_identity'
            if self.pending_count < (self.reacquire_confirmations if strong else self.confirmations):
                return None
            self.anchor, self.stamp = selected, stamp
            self.count = self.confirmations
            self.pending = self.pending_stamp = None
            self.pending_count = 0
            self.pending_strong = None
            self.reason, self.reacquired = 'tracking', True
            return selected
        if self.anchor is None:
            # Do not choose the largest/nearest arbitrary blob on acquisition.
            if len(candidates) != 1:
                return self.miss('ambiguous_target' if candidates else 'target_lost')
            selected = candidates[0]
        else:
            dt = stamp - self.stamp
            gate = min(self.reacquire_radius, self.jitter + self.speed * dt)
            ranked = sorted((distance(p, self.anchor), index, p)
                            for index, p in enumerate(candidates) if distance(p, self.anchor) <= gate)
            if not ranked:
                # Preserve identity anchor after loss; reset explicitly if relocalizing.
                return self.miss('target_lost')
            if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < self.margin:
                return self.miss('ambiguous_target')
            selected = ranked[0][2]
        self.anchor, self.stamp = selected, stamp
        self.count += 1
        self.locked = self.locked or self.count >= self.confirmations
        self.reason = 'tracking' if self.count >= self.confirmations else 'confirming_target'
        return selected if self.reason == 'tracking' else None


class Trail(object):
    def __init__(self, spacing=0.05, jump=0.5, length=15.0):
        self.spacing, self.jump, self.length = spacing, jump, length
        self.points = []

    def add(self, point):
        if not all(finite(v) for v in point[:2]):
            return False
        if self.points:
            delta = distance(self.points[-1], point)
            if delta > self.jump:
                return False
            if delta < self.spacing:
                return True
        self.points.append(tuple(point[:2]))
        length = sum(distance(a, b) for a, b in zip(self.points, self.points[1:]))
        while len(self.points) > 2 and length > self.length:
            length -= distance(self.points[0], self.points[1])
            self.points.pop(0)
        return True

    def behind(self, gap):
        for a, b in reversed(list(zip(self.points, self.points[1:]))):
            length = distance(a, b)
            if length >= gap:
                r = gap / length
                return (b[0] + r * (a[0] - b[0]), b[1] + r * (a[1] - b[1]),
                        math.atan2(b[1] - a[1], b[0] - a[0]))
            gap -= length
        return None


class Completion(object):
    """Done + stationary target + trailing distance + whole robot in finish zone."""
    def __init__(self, hold=2.0, motion_tolerance=0.08):
        self.hold, self.tolerance = hold, motion_tolerance
        self.anchor = self.since = None

    def reset(self):
        self.anchor = self.since = None

    def ready(self, mission, target, now, robot, goal, tolerance, polygon, radius, stopped=False):
        if (mission != 'done' or not stopped or distance(robot, goal) > tolerance or
                not circle_inside_polygon(robot, radius, polygon)):
            self.reset()
            return False
        if self.anchor is None or distance(target, self.anchor) > self.tolerance or now < self.since:
            self.anchor, self.since = target, now
        return (now - self.since >= self.hold and distance(robot, goal) <= tolerance and
                circle_inside_polygon(robot, radius, polygon))


def circle_inside_polygon(center, radius, polygon):
    if len(polygon) < 3:
        return False
    x, y = center[:2]
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if (a[1] > y) != (b[1] > y):
            if x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]:
                inside = not inside
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx * dx + dy * dy
        t = max(0., min(1., ((x - a[0]) * dx + (y - a[1]) * dy) / length2)) if length2 else 0.
        if math.hypot(x - a[0] - t * dx, y - a[1] - t * dy) < radius:
            return False
    return inside
