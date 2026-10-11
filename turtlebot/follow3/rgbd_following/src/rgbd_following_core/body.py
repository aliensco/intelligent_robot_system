"""Markerless proposals from visible floor geometry; identity still needs enrollment.

Assumes a roughly upright camera looking across a flat floor. Missing or
unreliable floor evidence produces no candidates, never a color fallback.
Floor removal applies ONLY to recognition, not to the collision point cloud.
"""
import cv2
import numpy as np
from .appearance import match_reference


def floor_plane(points, valid, rows):
    mask = valid & (rows >= rows.max() * .5)
    samples = points[mask]
    if len(samples) < 100:
        return None
    step = max(1, len(samples) // 1000)
    samples = samples[::step]
    pixel_y, pixel_x = np.nonzero(mask)
    pixel_y, pixel_x = pixel_y[::step], pixel_x[::step]
    rng = np.random.RandomState(7)
    triplets = samples[rng.randint(len(samples), size=(96, 3))]
    normals = np.cross(triplets[:, 1] - triplets[:, 0], triplets[:, 2] - triplets[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(lengths[:, None], 1e-8)
    normals[normals[:, 1] > 0] *= -1
    offsets = -np.sum(normals * triplets[:, 0], axis=1)
    good = (lengths > 1e-6) & (normals[:, 1] < -.75) & (offsets > .15) & (offsets < 1.2)
    if not np.any(good):
        return None
    normals, offsets = normals[good], offsets[good]
    # Broadcasting avoids small-matrix BLAS thread overhead on older Ubuntu.
    residuals = np.abs(np.sum(samples[:, None, :] * normals[None, :, :], axis=2) + offsets)
    support = residuals < .02
    best = int(np.argmax(support.sum(axis=0)))
    if support[:, best].mean() < .25:
        return None
    accepted = support[:, best]
    for unused in range(3):
        inliers = samples[accepted]
        center = inliers.mean(axis=0)
        normal = np.linalg.svd(inliers - center, full_matrices=False)[2][-1]
        if normal[1] > 0:
            normal = -normal
        offset = -float(np.dot(normal, center))
        accepted = np.abs(np.sum(samples * normal, axis=1) + offset) < .02
        if accepted.sum() < 100 or accepted.mean() < .25:
            return None
    if normal[1] >= -.75 or not .15 < offset < 1.2:
        return None
    # Clutter may occupy most lower pixels. Instead of requiring a majority,
    # demand distributed floor evidence across the image, close to its bottom,
    # and at multiple depths. A small tabletop/obstacle patch is insufficient.
    if (np.diff(np.percentile(pixel_x[accepted], [10, 90]))[0] < points.shape[1] * .35 or
            np.percentile(pixel_y[accepted], 90) < points.shape[0] * .8 or
            np.diff(np.percentile(samples[accepted, 2], [10, 90]))[0] < .20):
        return None
    return normal, offset


def body_feature(hsv, support, bbox, depth, camera):
    x, y, w, h = bbox
    histogram = []
    for ya, yb in ((y, y+h//2), (y+h//2, y+h)):
        for xa, xb in ((x, x+w//2), (x+w//2, x+w)):
            pixels = hsv[ya:yb, xa:xb][support[ya:yb, xa:xb]]
            # Achromatic colors use brightness, not unstable near-gray hue.
            bins = np.where(pixels[:, 1] < 60, pixels[:, 2].astype(int)*12//256,
                            12 + pixels[:, 0].astype(int)*12//180*2 + (pixels[:, 1] >= 160))
            hist = np.bincount(bins, minlength=36).astype(float)
            histogram.extend((hist / max(1., hist.sum())).tolist())
    silhouette = np.clip(cv2.resize(support[y:y+h, x:x+w].astype(np.float32), (8, 8),
                                    interpolation=cv2.INTER_AREA), 0., 1.).ravel().tolist()
    return dict(kind='depth_body_v1', histogram=histogram, silhouette=silhouette,
                aspect=float(w)/h, width_m=w*depth/camera[0], height_m=h*depth/camera[1])


def floor_plane_for_depth(depth, camera, config):
    """Fit one ground plane for all identity branches of the same RGB-D pair."""
    fx, fy, cx, cy = camera
    rows, cols = np.mgrid[0:depth.shape[0]:4, 0:depth.shape[1]:4]
    z = depth[::4, ::4]
    finite = np.isfinite(z)
    safe = np.where(finite, z, 0.)
    valid = finite & (safe >= config['min_depth']) & (safe <= config['max_depth'])
    safe = np.where(valid, safe, 0.)
    points = np.stack(((cols-cx)*safe/fx, (rows-cy)*safe/fy, safe), axis=-1)
    return floor_plane(points, valid, rows)


def detect_body(rgb, depth, camera, config, plane=False):
    options = config['body']
    stride = 4
    rows, cols = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    finite = np.isfinite(z)
    safe_z = np.where(finite, z, 0.)
    valid = finite & (safe_z >= config['min_depth']) & (safe_z <= config['max_depth'])
    fx, fy, cx, cy = camera
    points = np.stack(((cols-cx)*safe_z/fx, (rows-cy)*safe_z/fy, safe_z), axis=-1)
    if plane is False:
        plane = floor_plane(points, valid, rows)
    empty = np.zeros(depth.shape, dtype=np.uint8)
    if plane is None:
        return [], empty
    heights = np.sum(points * plane[0], axis=2) + plane[1]
    # An optional ground-relative band can isolate a wheeled base from its
    # disconnected shelves. This affects recognition only, never obstacle clouds.
    support_height = options.get('max_support_height_m', options['max_height_m'] + .08)
    foreground = valid & (heights > .035) & (heights < support_height)
    proposals = []
    for low in np.arange(config['min_depth'], config['max_depth'], .15):
        slab = foreground & (safe_z >= low) & (safe_z < low + .30)
        closed = cv2.morphologyEx(slab.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        for contour in cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
            component = np.zeros(closed.shape, dtype=np.uint8)
            cv2.drawContours(component, [contour], -1, 1, -1)
            support = (component > 0) & slab
            if support.sum() < 30:
                continue
            v, u = np.nonzero(support)
            x, y, w, h = cv2.boundingRect(contour)
            distance = float(np.median(z[support]))
            width, height = w*stride*distance/fx, h*stride*distance/fy
            if not (options['min_width_m'] <= width <= options['max_width_m'] and
                    options['min_height_m'] <= height <= options['max_height_m']):
                continue
            # Do not accept cropped bodies or floating background fragments.
            if x == 0 or y == 0 or x+w >= z.shape[1] or y+h >= z.shape[0]:
                continue
            if np.percentile(heights[support], 10) > .11:
                continue
            proposals.append((int(support.sum()), (x*stride, y*stride, w*stride, h*stride),
                              distance, support, float(np.median(u))*stride, float(np.median(v))*stride))
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    candidates = []
    # Score EVERY depth window before suppression. A larger, contaminated
    # window must not discard an overlapping window that matches the target.
    for _, bbox, distance, support, u, v in proposals:
        full = np.repeat(np.repeat(support, stride, axis=0), stride, axis=1)[:depth.shape[0], :depth.shape[1]]
        full &= np.isfinite(depth) & (np.abs(np.nan_to_num(depth)-distance) < .30)
        feature = body_feature(hsv, full, bbox, distance, camera)
        identity = config['identity']
        score = match_reference(feature, identity['references'], identity['shape_ratio'], identity['width_ratio'])
        if identity['references'] and score > identity['max_distance']:
            continue
        candidates.append(dict(point=((u-cx)*distance/fx, (v-cy)*distance/fy, distance),
                               bbox=bbox, depth=distance, feature=feature, identity_score=score,
                               identity_verified=bool(identity['references']) and score <= identity['reacquire_distance'],
                               valid_ratio=float(full[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]].mean()),
                               _support=full))
    selected = []
    for candidate in sorted(candidates, key=lambda c: (c['identity_score'],
                             -int(c['_support'].sum()))):
        x, y, w, h = candidate['bbox']
        duplicate = False
        for old in selected:
            a, b, c, d = old['bbox']
            intersection = max(0, min(x+w, a+c)-max(x, a))*max(0, min(y+h, b+d)-max(y, b))
            # A depth window can produce a small nested fragment of the SAME
            # measured body. IoU alone misses it. Shared actual depth pixels
            # distinguish that fragment from separate nearby robots.
            shared = int((candidate['_support'] & old['_support']).sum())
            smaller = min(int(candidate['_support'].sum()), int(old['_support'].sum()))
            if (abs(candidate['depth']-old['depth']) < .15 and
                    (intersection / float(w*h+c*d-intersection) > .45 or
                     (smaller > 0 and shared / float(smaller) > .65))):
                duplicate = True
                break
        if not duplicate:
            selected.append(candidate)
    for candidate in selected:
        empty[candidate.pop('_support')] = 255
    return selected, empty
