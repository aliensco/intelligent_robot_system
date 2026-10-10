"""Short-horizon camera projection for a stationary observed target (Py2/3)."""
import math
from .depth import finite


def rollout(pose, v, w, horizon, step=.05):
    count = max(1, int(math.ceil(horizon / step)))
    dt = horizon / count
    x, y, yaw = pose
    for unused in range(count + 1):
        yield x, y, yaw
        if abs(w) < 1e-8:
            x, y = x + v*math.cos(yaw)*dt, y + v*math.sin(yaw)*dt
        else:
            x += v/w*(math.sin(yaw+w*dt)-math.sin(yaw))
            y -= v/w*(math.cos(yaw+w*dt)-math.cos(yaw))
        yaw += w*dt


def visible_path(pose, target, v, w, camera, optical_from_base, horizon, margin,
                 recovery=False, require_progress=True):
    """Check the observed target through a motion rollout.

    Recovery only relaxes an already violated inner margin: every violation
    must monotonically decrease, no new one may appear, and the target must
    stay inside the actual image (with a one-pixel border). The gate permits
    this mode only for in-place recentering of a fresh, confirmed target.
    A measured stopping rollout may hold its starting projection; a commanded
    recovery must make progress. This is not a search for an unseen target.
    """
    fx, fy, cx, cy, width, height = camera
    t, q = optical_from_base
    values = tuple(pose)+tuple(target)+(v,w)+tuple(camera)+tuple(t)+tuple(q)+(horizon,margin)
    if not all(finite(x) for x in values) or not 0 < margin < .45:
        return False
    if abs(sum(x*x for x in q)-1.) > .05 or min(fx,fy,width,height,horizon) <= 0:
        return False
    qx, qy, qz, qw = q
    initial = previous = None
    for x, y, heading in rollout(pose, v, w, horizon):
        dx, dy = target[0]-x, target[1]-y
        bx, by, bz = math.cos(heading)*dx+math.sin(heading)*dy, -math.sin(heading)*dx+math.cos(heading)*dy, target[2]
        # Unit quaternion rotation: optical <- base, including measured mount.
        ux, uy, uz = 2*(qy*bz-qz*by), 2*(qz*bx-qx*bz), 2*(qx*by-qy*bx)
        px = bx+qw*ux+qy*uz-qz*uy+t[0]
        py = by+qw*uy+qz*ux-qx*uz+t[1]
        pz = bz+qw*uz+qx*uy-qy*ux+t[2]
        if pz <= 0:
            return False
        u, vv = fx*px/pz+cx, fy*py/pz+cy
        if recovery:
            if not 1 <= u <= width-2 or not 1 <= vv <= height-2:
                return False
            violation = (max(0., margin-u/width), max(0., u/width-(1-margin)),
                         max(0., margin-vv/height), max(0., vv/height-(1-margin)))
            if previous is not None and any(a > b+1e-9 for a,b in zip(violation, previous)):
                return False
            if initial is None:
                initial = violation
            previous = violation
        elif not margin*width <= u <= (1-margin)*width or not margin*height <= vv <= (1-margin)*height:
            return False
    return not recovery or not require_progress or sum(previous) < sum(initial)-1e-6
