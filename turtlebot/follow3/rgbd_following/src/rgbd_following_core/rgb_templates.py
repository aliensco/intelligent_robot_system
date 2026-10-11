"""Fixed RGB templates backed by measured body-depth pixels, no online learning."""
import cv2
import numpy as np
from .body import floor_plane


def detect_templates(rgb, depth, camera, config):
    templates = config.get('rgb_templates', [])
    mask = np.zeros(depth.shape, np.uint8)
    if not templates:
        return [], mask
    fx, fy, cx, cy = camera
    rows, cols = np.mgrid[0:depth.shape[0]:4, 0:depth.shape[1]:4]
    z = depth[::4, ::4]
    bounded_z = np.where(np.isfinite(z), z, 0.)
    valid = np.isfinite(z) & (bounded_z >= config['min_depth']) & (bounded_z <= config['max_depth'])
    safe = np.where(valid, z, 0.)
    points = np.stack(((cols-cx)*safe/fx, (rows-cy)*safe/fy, safe), axis=-1)
    plane = floor_plane(points, valid, rows)
    if plane is None:
        return [], mask
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    proposals = []
    for reference in templates:
        original = np.asarray(reference['pixels'], np.uint8)
        if original.ndim != 2 or min(original.shape) < 8 or original.std() < 8:
            raise ValueError('RGB template must have visible texture')
        aspect = reference['aspect']
        for width in (40, 48, 56, 64, 76, 88, 104, 120, 140, 164, 192, 224):
            height = int(round(width/aspect))
            if height < 8 or width >= gray.shape[1] or height >= gray.shape[0]:
                continue
            template = cv2.resize(original, (width, height), interpolation=cv2.INTER_LINEAR)
            score = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
            # Keep multiple local maxima. A second real matching robot must
            # remain a competing identity rather than silently pick the best.
            for unused in range(4):
                _, peak, _, position = cv2.minMaxLoc(score)
                if peak < reference.get('min_correlation', .78):
                    break
                x, y = position
                score[max(0,y-height//2):min(score.shape[0],y+height//2+1),
                      max(0,x-width//2):min(score.shape[1],x+width//2+1)] = -1
                patch = depth[y:y+height, x:x+width]
                vv, uu = np.mgrid[y:y+height, x:x+width]
                bounded_patch = np.where(np.isfinite(patch), patch, 0.)
                finite = np.isfinite(patch) & (bounded_patch >= config['min_depth']) & (bounded_patch <= config['max_depth'])
                zz = np.where(finite, patch, 0.)
                h = ((uu-cx)*zz/fx*plane[0][0] + (vv-cy)*zz/fy*plane[0][1] + zz*plane[0][2]+plane[1])
                support = finite & (h > .035) & (h < .20)
                samples = patch[support]
                if len(samples) < config['min_depth_samples']:
                    continue
                measured = float(np.median(samples))
                same_depth = support & (np.abs(zz-measured) <= .10)
                if same_depth.sum() < config['min_depth_samples']:
                    continue
                if not .5*reference['width_m'] <= width*measured/fx <= 1.8*reference['width_m']:
                    continue
                if not .04 <= height*measured/fy <= .25:
                    continue
                proposals.append(dict(bbox=(x,y,width,height), depth=measured,
                    point=((x+width/2.-cx)*measured/fx,(y+height/2.-cy)*measured/fy,measured),
                    identity_score=1.-peak, identity_verified=peak >= reference.get('verified_correlation', .9),
                    feature=dict(kind='rgb_template_v1', label=reference['label']),
                    valid_ratio=float(same_depth.mean()), _correlation=peak))
    chosen = []
    for candidate in sorted(proposals, key=lambda c: -c['_correlation']):
        x,y,w,h = candidate['bbox']
        duplicate = False
        for old in chosen:
            a,b,c,d = old['bbox']
            intersection = max(0,min(x+w,a+c)-max(x,a))*max(0,min(y+h,b+d)-max(y,b))
            if abs(candidate['depth']-old['depth']) < .15 and intersection/float(min(w*h,c*d)) > .6:
                duplicate = True
                break
        if not duplicate:
            candidate.pop('_correlation')
            chosen.append(candidate)
            mask[y:y+h,x:x+w] = 255
    return chosen, mask
