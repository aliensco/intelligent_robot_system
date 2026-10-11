"""Reachable follow endpoints and temporary detour endpoints on a costmap."""
import math
import heapq
import cv2
import numpy as np


def _approach_angle(path, target, horizon, window=.10):
    """Largest local path/target bearing difference over a measured route.

    A single start/end chord hides turns in an S-shaped approach. Sliding short
    chords smooth grid stair steps while retaining those turns. Target bearing
    is evaluated at each chord's midpoint, since translating near the leader
    also changes its bearing. This is a ranking cost, not camera visibility or
    occlusion proof; the independent camera/velocity checks still apply.
    """
    if len(path) < 2:
        return 0.
    samples = [(path[0][0], path[0][1], 0.)]
    for point in path[1:]:
        previous = samples[-1]
        segment = math.hypot(point[0]-previous[0], point[1]-previous[1])
        if segment <= 1e-9:
            continue
        used = min(segment, horizon-previous[2])
        fraction = used/segment
        samples.append((previous[0]+fraction*(point[0]-previous[0]),
                        previous[1]+fraction*(point[1]-previous[1]),
                        previous[2]+used))
        if samples[-1][2] >= horizon:
            break
    total = samples[-1][2]
    if total <= 1e-9:
        return 0.
    worst, start = 0., 0
    for end in range(1, len(samples)):
        b = samples[end]
        if b[2]+1e-9 < min(window, total):
            continue
        begin = max(0., b[2]-window)
        while start+1 < end and samples[start+1][2] < begin:
            start += 1
        a, after = samples[start], samples[start+1]
        fraction = (begin-a[2])/(after[2]-a[2])
        ax, ay = a[0]+fraction*(after[0]-a[0]), a[1]+fraction*(after[1]-a[1])
        if math.hypot(b[0]-ax, b[1]-ay) <= 1e-9:
            continue
        direction = math.atan2(b[1]-ay, b[0]-ax)
        bearing = math.atan2(target[1]-(ay+b[1])/2., target[0]-(ax+b[0])/2.)
        turn = direction-bearing
        worst = max(worst, abs(math.atan2(math.sin(turn), math.cos(turn))))
    return worst


def reachable_standoff(grid, robot, target, desired, minimum, radius,
                      extra_distance=0., excluded=None, path_cost_weight=.25,
                      heading_cost_weight=.8, max_expansions=12000, path_lookahead=.30):
    """Rank normal follow endpoints and temporary waypoints by usable approaches.

    Unknown and lethal cells exclude the full footprint; inscribed inflation
    excludes only the centre. Eight-connected Dijkstra distances (without corner
    cutting) score the actual approach, including soft inflation, instead of
    preferring an endpoint whose straight-line distance hides a long detour.
    A temporary waypoint may be farther from the target to leave a dead end.
    This selects a goal, never replaces move_base's actual path or velocity gate.

    ``path_cost_weight`` is the additional cost per metre at maximum soft
    inflation, not extra footprint padding. Normal and farther endpoints compete
    with a gap-error penalty, allowing an easy forward detour to beat a nominal
    gap that requires first turning away. ``heading_cost_weight`` penalizes the
    largest local heading/target-bearing difference along the first 0.25 m
    (or longer ``path_lookahead``) of the reconstructed approach. Short sliding
    chords retain turns hidden by the whole prefix's start/end direction.
    This soft cost does not prohibit a necessary turn away or prove visibility.
    ``excluded`` accepts one point or a sequence of recently
    failed/visited points; their lifetime is managed by the caller.
    The route-cost search is bounded by ``max_expansions`` to keep the live
    controller responsive. If that budget is exhausted before reaching a goal,
    a reached forward intermediate point with a reconstructed approach can be
    offered instead. No geometric-only fallback bypasses the approach ranking.
    ``path_lookahead`` bounds long approaches to a waypoint on the selected
    route, so move_base cannot replace the selected nearby approach with an
    unrelated route to a distant endpoint. Zero returns the full endpoint for
    diagnostics; the default advances about 0.30 m along the actual path.
    """
    if (not 0 < minimum <= desired or extra_distance < 0 or
            not 0 <= path_cost_weight <= 10 or not 0 <= heading_cost_weight <= 10 or
            max_expansions < 1 or not 0 <= path_lookahead <= 1.):
        raise ValueError('Invalid follow distance interval')
    if not grid.clear(robot[0], robot[1], radius):
        return None
    resolution = grid.resolution
    values = np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)
    pad = radius+resolution*math.sqrt(2)/2.
    cells = int(math.ceil(pad/resolution))
    yy, xx = np.mgrid[-cells:cells+1, -cells:cells+1]
    kernel = ((xx*resolution)**2+(yy*resolution)**2 <= pad**2).astype(np.uint8)
    occupied = ((values < 0) | (values >= 100)).astype(np.uint8)
    inflated = cv2.dilate(occupied, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=1)
    reachable = ((inflated == 0) & (values >= 0) & (values < 99)).astype(np.uint8)
    c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])

    def cell(point):
        dx, dy = point[0]-grid.origin[0], point[1]-grid.origin[1]
        return int(math.floor((c*dx+s*dy)/resolution)), int(math.floor((-s*dx+c*dy)/resolution))

    seed = cell(robot)
    if not (0 <= seed[0] < grid.width and 0 <= seed[1] < grid.height) or not reachable[seed[1], seed[0]]:
        return None
    heading = math.atan2(target[1]-robot[1], target[0]-robot[0])
    excluded_points = []
    if excluded is not None and len(excluded):
        excluded_points = excluded if isinstance(excluded[0], (tuple, list, np.ndarray)) else [excluded]
    normal_count = max(1, int(math.ceil((desired-minimum)/.05)))
    gaps = [(desired-(desired-minimum)*i/normal_count, False) for i in range(normal_count+1)]
    # Do not require every intermediate endpoint to decrease target distance:
    # obstacle layouts can require a side step or a temporary retreat. Actual
    # target-distance stopping and footprint checks remain independent.
    upper = desired+extra_distance
    steps = max(0, int(math.ceil((upper-desired)/.10)))
    gaps += [(desired+(upper-desired)*i/steps, True) for i in range(1, steps+1)]
    choices = {}
    for gap, detour in gaps:
        for degrees in (0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90):
            angle = heading+math.radians(degrees)
            x, y = target[0]-gap*math.cos(angle), target[1]-gap*math.sin(angle)
            ix, iy = cell((x, y))
            if not (0 <= ix < grid.width and 0 <= iy < grid.height) or not reachable[iy, ix]:
                continue
            if any(math.hypot(x-previous[0], y-previous[1]) < .15 for previous in excluded_points):
                continue
            if not grid.clear(x, y, radius):
                continue
            if detour and math.hypot(x-robot[0], y-robot[1]) < .10:
                continue
            score = 2*abs(gap-desired)+.05*abs(math.radians(degrees))
            choices.setdefault((ix, iy), []).append(
                (detour, score, (x, y, math.atan2(target[1]-y, target[0]-x))))
    if not choices:
        return None

    # Cheap connectivity filtering avoids exploring a whole map for a desired
    # ring that is separated by a wall. Four-way fill matches the connectivity
    # of eight-way traversal that disallows diagonal corner cutting.
    cv2.floodFill(reachable, None, seed, 2, flags=4)
    choices = dict((key, rows) for key, rows in choices.items()
                   if reachable[key[1], key[0]] == 2)
    if not choices:
        return None
    # Lists in the inner search loop avoid NumPy scalar indexing overhead on
    # Kinetic/Python 2. NumPy/OpenCV still handle the bulk footprint operation.
    width, height = grid.width, grid.height
    reachable_cells = reachable.ravel().tolist()
    cell_values = values.ravel().tolist()
    costs = [float('inf')]*(width*height)
    parents = [-1]*(width*height)
    seed_index = seed[1]*width+seed[0]
    costs[seed_index] = 0.
    queue = [(0., seed_index)]
    endpoint_rows = {}
    for (ix, iy), rows in choices.items():
        dx, dy = (ix+.5)*resolution, (iy+.5)*resolution
        center = (grid.origin[0]+c*dx-s*dy, grid.origin[1]+s*dx+c*dy)
        endpoint_rows[iy*width+ix] = [
            (detour, penalty+math.hypot(point[0]-center[0], point[1]-center[1]), point)
            for detour, penalty, point in rows]
    steps = ((1, 0, 1.), (-1, 0, 1.), (0, 1, 1.), (0, -1, 1.),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
             (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))
    best = None
    inflation_scale = path_cost_weight/196.
    expanded = 0
    settled = []

    def approach_penalty(index):
        if not heading_cost_weight:
            return 0.
        route = []
        while index != seed_index:
            route.append(index)
            index = parents[index]
        path = [(robot[0], robot[1])]
        length = 0.
        horizon = max(.25, path_lookahead)
        for index in reversed(route):
            gx, gy = (index % width+.5)*resolution, (index // width+.5)*resolution
            point = (grid.origin[0]+c*gx-s*gy, grid.origin[1]+s*gx+c*gy)
            length += math.hypot(point[0]-path[-1][0], point[1]-path[-1][1])
            path.append(point)
            if length >= horizon:
                break
        return heading_cost_weight*_approach_angle(path, target, horizon)

    def route_waypoint(index, point, detour):
        if not path_lookahead:
            return point, detour
        route = []
        while index != seed_index:
            route.append(index)
            index = parents[index]
        length = 0.
        previous = seed_index
        intermediate = []
        for index in reversed(route):
            ix, iy = index % width, index // width
            length += resolution*math.hypot(ix-previous % width, iy-previous // width)
            if max(.10, path_lookahead-.05) <= length <= path_lookahead+.05:
                gx, gy = (ix+.5)*resolution, (iy+.5)*resolution
                x, y = grid.origin[0]+c*gx-s*gy, grid.origin[1]+s*gx+c*gy
                if (math.hypot(x-target[0], y-target[1]) >= minimum and
                        not any(math.hypot(x-p[0], y-p[1]) < .15 for p in excluded_points) and
                        grid.clear(x, y, radius)):
                    intermediate.append((abs(length-path_lookahead),
                        (x, y, math.atan2(target[1]-y, target[0]-x))))
            previous = index
        if length > path_lookahead+.15:
            # Skip this candidate if every local point is excluded/blocked,
            # so another approach can win instead of bypassing an exclusion.
            if not intermediate:
                return None
            return min(intermediate, key=lambda row: row[0])[1], True
        return point, detour

    while queue and expanded < max_expansions:
        cost, index = heapq.heappop(queue)
        if cost > costs[index]:
            continue
        expanded += 1
        settled.append(index)
        if best is not None and cost > best[1]:
            break  # Endpoint penalties are nonnegative.
        rows = endpoint_rows.get(index, ())
        direction_penalty = approach_penalty(index) if rows else 0.
        for detour, penalty, point in rows:
            score = cost+penalty+direction_penalty
            if best is None or score < best[1]:
                local = route_waypoint(index, point, detour)
                if local is not None:
                    best = (local[1], score, local[0])
        ix, iy = index % width, index // width
        for ox, oy, length in steps:
            nx, ny = ix+ox, iy+oy
            neighbor = index+oy*width+ox
            if not (0 <= nx < width and 0 <= ny < height) or reachable_cells[neighbor] != 2:
                continue
            if ox and oy and (reachable_cells[index+ox] != 2 or reachable_cells[index+oy*width] != 2):
                continue
            inflation = (cell_values[index]+cell_values[neighbor])*inflation_scale
            candidate_cost = cost+length*resolution*(1.+inflation)
            if candidate_cost < costs[neighbor]:
                costs[neighbor] = candidate_cost
                parents[neighbor] = index
                heapq.heappush(queue, (candidate_cost, neighbor))
    if best is None:
        # A budget fallback still needs an actually reconstructed approach.
        # Restrict it to local forward progress, retain a bounded shortlist,
        # then apply the same path-heading cost as for the distance-ring goals.
        fallback = []
        forward_x, forward_y = math.cos(heading), math.sin(heading)
        for index in settled:
            gx, gy = (index % width+.5)*resolution, (index // width+.5)*resolution
            x, y = grid.origin[0]+c*gx-s*gy, grid.origin[1]+s*gx+c*gy
            dx, dy = x-robot[0], y-robot[1]
            if not .10**2 <= dx*dx+dy*dy <= .50**2 or dx*forward_x+dy*forward_y < .05:
                continue
            gap = math.hypot(x-target[0], y-target[1])
            if gap < minimum or any(math.hypot(x-p[0], y-p[1]) < .15 for p in excluded_points):
                continue
            fallback.append((costs[index]+2*abs(gap-desired), index,
                             (x, y, math.atan2(target[1]-y, target[0]-x))))
        for score, index, point in heapq.nsmallest(16, fallback):
            if not grid.clear(point[0], point[1], radius):
                continue
            score += approach_penalty(index)
            if best is None or score < best[1]:
                local = route_waypoint(index, point, True)
                if local is not None:
                    best = (local[1], score, local[0])
    if best is None:
        return None
    return best[2], best[0]
