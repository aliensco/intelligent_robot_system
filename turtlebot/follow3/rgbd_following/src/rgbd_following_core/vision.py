"""Calibrated color signature + registered depth. Not a generic TonyPi model."""
import cv2
import numpy as np
from .depth import finite, summarize
from .appearance import match_reference


def appearance_feature(hsv, depth, bbox, z, width_m, band):
    # Include nearby colors at the same depth, not just the seed color contour.
    x, y, w, h = bbox
    x0, x1 = max(0, x-w//2), min(hsv.shape[1], x+w+w//2)
    y0, y1 = max(0, y-h//2), min(hsv.shape[0], y+h+h//2)
    histogram = []
    for ya, yb in ((y0, (y0+y1)//2), ((y0+y1)//2, y1)):
        for xa, xb in ((x0, (x0+x1)//2), ((x0+x1)//2, x1)):
            d = depth[ya:yb, xa:xb]
            mask = (np.isfinite(d) & (np.abs(d-z) <= band)).astype(np.uint8)
            hist = cv2.calcHist([hsv[ya:yb, xa:xb]], [0, 1], mask, [12, 3], [0, 180, 0, 256]).ravel()
            hist /= max(float(hist.sum()), 1.)
            histogram.extend(float(v) for v in hist)
    return dict(histogram=histogram, aspect=float(w)/h, width_m=float(width_m))


def intrinsics(projection, width, height):
    if (len(projection) != 12 or not all(finite(v) for v in projection) or
            projection[0] <= 0 or projection[5] <= 0 or
            not 0 <= projection[2] < width or not 0 <= projection[6] < height or
            abs(projection[3]) > 1e-6 or abs(projection[7]) > 1e-6):
        raise ValueError('invalid_rectified_rgb_calibration')
    return projection[0], projection[5], projection[2], projection[6]


def metric_depth(depth, encoding):
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('unsupported_depth_encoding')
    return depth.astype(np.float32) * (0.001 if encoding == '16UC1' else 1.0)


def cloud_points(depth_m, camera, stride=4, near=0.4, far=4.0):
    fx, fy, cx, cy = camera
    rows, cols = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    z = depth_m[::stride, ::stride]
    finite_z = np.isfinite(z)
    safe_z = np.where(finite_z, z, 0.)
    good = finite_z & (safe_z >= near) & (safe_z <= far)
    z, u, v = z[good], cols[good], rows[good]
    return np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z)).astype(np.float32)


def observation_clouds(depth_m, camera, perception):
    """Separate measured obstacle endpoints from finite clearing endpoints.

    Neither stream invents returns for zero/NaN/Inf pixels. The longer clearing
    range is a sensor setting, not a replacement for invalid/background depth.
    Old enrolled configurations retain their original range unless updated.
    """
    p = perception
    obstacles = cloud_points(depth_m, camera, p['cloud_stride'], p['min_depth'], p['max_depth'])
    clearing = cloud_points(depth_m, camera, p.get('clearing_stride', p['cloud_stride']),
                            p['min_depth'], p.get('clearing_max_depth', p['max_depth']))
    return obstacles, clearing


def transform_points(points, translation, quaternion):
    t, q = np.asarray(translation, dtype=float), np.asarray(quaternion, dtype=float)
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q) - 1.) > .05:
        raise ValueError('invalid_transform')
    q /= np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return np.asarray(points).reshape((-1, 3)).dot(rotation.T) + t


def detect(rgb, depth_m, camera, config):
    """Return independent candidates; tracker handles association and ambiguity."""
    if rgb.shape[:2] != depth_m.shape:
        raise ValueError('registered_size_mismatch')
    if config.get('proposal_mode', 'color') == 'depth_body':
        from .body import detect_body
        candidates, mask = detect_body(rgb, depth_m, camera, config)
        if not candidates and config.get('rgb_templates'):
            from .rgb_templates import detect_templates
            return detect_templates(rgb, depth_m, camera, config)
        return candidates, mask
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    mask = np.zeros(depth_m.shape, dtype=np.uint8)
    for band in config['hsv_bands']:
        mask |= cv2.inRange(hsv, np.array(band['lower'], dtype=np.uint8), np.array(band['upper'], dtype=np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    fx, fy, cx, cy = camera
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not config['min_area_ratio'] <= area / mask.size <= config['max_area_ratio']:
            continue
        component = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(component, [contour], -1, 255, -1)
        # Shrink edges to avoid depth registration holes and mixed background.
        component = cv2.erode(component & mask, kernel, iterations=1)
        values = depth_m[component > 0]
        summary = summarize(values, '32FC1', min_depth=config['min_depth'],
                            max_depth=config['max_depth'], min_samples=config['min_depth_samples'],
                            min_ratio=config['min_valid_ratio'], max_spread=config['max_depth_spread'])
        if not summary['usable']:
            continue
        z = summary['median_m']
        good = ((component > 0) & np.isfinite(depth_m) &
                (np.abs(depth_m - z) <= config['depth_cluster_band']))
        vs, us = np.nonzero(good)
        if len(us) < config['min_depth_samples']:
            continue
        x0, y0, width, height = cv2.boundingRect(contour)
        physical_width = width * z / fx
        if not config['min_width_m'] <= physical_width <= config['max_width_m']:
            continue
        u, v = float(np.median(us)), float(np.median(vs))
        point = ((u - cx) * z / fx, (v - cy) * z / fy, z)
        feature = appearance_feature(hsv, depth_m, (x0, y0, width, height), z,
                                     physical_width, config['depth_cluster_band'])
        identity = config['identity']
        score = match_reference(feature, identity['references'], identity['shape_ratio'], identity['width_ratio'])
        if identity['references'] and score > identity['max_distance']:
            continue
        candidates.append(dict(point=point, bbox=(x0, y0, width, height), depth=z,
                               feature=feature, identity_score=score,
                               identity_verified=bool(identity['references']) and score <= identity['reacquire_distance'],
                               valid_ratio=summary['valid_ratio']))
    return candidates, mask
