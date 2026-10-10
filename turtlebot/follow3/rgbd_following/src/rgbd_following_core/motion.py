"""Grid collision checks and watchdog decisions, independent of ROS."""
import math
from .depth import finite


def fresh(stamp, now, timeout):
    return stamp is not None and finite(stamp) and 0 <= now - stamp <= timeout


def stopping_clear(grid, pose, measured_speed, requested_speed, radius, braking, reaction):
    """All-direction stopping envelope, including inertia after a zero command.

    Assumes speed cannot rise above max(measured, requested) during reaction and
    measured braking is a conservative lower bound. Does not assume curvature
    stays constant as angular and linear velocities decay independently.
    """
    if not all(finite(x) for x in (measured_speed, requested_speed, radius, braking, reaction)):
        return False
    if min(measured_speed, requested_speed, reaction) < 0 or braking <= 0:
        return False
    speed = max(measured_speed, requested_speed)
    reach = speed*reaction + speed*speed/(2*braking)
    return grid.clear(pose[0], pose[1], radius+reach)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Grid(object):
    """OccupancyGrid published by ROS costmap_2d, not a probability map.

    The publisher maps lethal obstacles to 100, inscribed inflation to 99,
    soft inflation costs to 1..98 and unknown space to -1. Inflation describes
    forbidden/costly ROBOT CENTER positions, not additional physical obstacles.
    """
    def __init__(self, data, width, height, resolution, origin=(0., 0., 0.)):
        if (width <= 0 or height <= 0 or not finite(resolution) or resolution <= 0 or
                len(data) != width * height or not all(finite(v) for v in origin)):
            raise ValueError('Invalid costmap')
        self.data, self.width, self.height = data, width, height
        self.resolution, self.origin = resolution, origin

    def clear(self, x, y, radius):
        """Check a center exclusion and one explicit circular footprint.

        Retain the map's inscribed center exclusion (including its padding),
        then test the full footprint/envelope against lethal and unknown cells.
        Applying the footprint to inflated 99-valued cells would count the
        robot radius twice and incorrectly reject otherwise passable corridors.
        """
        if not all(finite(v) for v in (x, y, radius)) or radius <= 0:
            return False
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        gx, gy = c * dx + s * dy, -s * dx + c * dy
        pad = radius + self.resolution * math.sqrt(2) / 2
        x0, x1 = int(math.floor((gx - pad) / self.resolution)), int(math.floor((gx + pad) / self.resolution))
        y0, y1 = int(math.floor((gy - pad) / self.resolution)), int(math.floor((gy + pad) / self.resolution))
        if x0 < 0 or y0 < 0 or x1 >= self.width or y1 >= self.height:
            return False
        center_x, center_y = int(math.floor(gx / self.resolution)), int(math.floor(gy / self.resolution))
        center_value = self.data[center_y * self.width + center_x]
        if center_value < 0 or center_value >= 99:
            return False
        for iy in range(y0, y1 + 1):
            for ix in range(x0, x1 + 1):
                if math.hypot((ix + .5) * self.resolution - gx, (iy + .5) * self.resolution - gy) <= pad:
                    value = self.data[iy * self.width + ix]
                    if value < 0 or value >= 100:
                        return False
        return True


def swept_clear(grid, pose, v, w, radius, braking=0.3, reaction=0.3):
    """Check circular swept footprint through reaction + conservative stop time.

    Constant speed until stop overestimates travel during deceleration. A circular
    bounding footprint also covers in-place rotation; it must include protrusions.
    """
    if not all(finite(x) for x in tuple(pose) + (v, w, radius, braking, reaction)) or v < 0 or braking <= 0:
        return False
    x, y, heading = pose
    duration = reaction + v / braking
    dt = min(.05, grid.resolution / max(2 * v, .001))
    count = max(1, int(math.ceil(duration / dt)))
    dt = duration / count
    for unused in range(count + 1):
        if not grid.clear(x, y, radius):
            return False
        x += v * math.cos(heading + w * dt / 2) * dt
        y += v * math.sin(heading + w * dt / 2) * dt
        heading += w * dt
    return True


def need_goal(previous, point, state, final, elapsed, shift=0.15, interval=0.5):
    if previous is None:
        return True
    delta = math.hypot(previous[0] - point[0], previous[1] - point[1])
    # Always deliver the final small shift; do not strand a robot at the old goal.
    return elapsed >= interval and delta >= (0.02 if final or state == 3 else shift)
