"""Fixed RGB templates backed by measured body-depth pixels, no online learning."""
import cv2
import math
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
    half_gray = cv2.resize(gray, (gray.shape[1]//2, gray.shape[0]//2),
                           interpolation=cv2.INTER_AREA)
    proposals = []
    for reference in templates:
        original = np.asarray(reference['pixels'], np.uint8)
        if original.ndim != 2 or min(original.shape) < 8 or original.std() < 8:
            raise ValueError('RGB template must have visible texture')
        aspect = reference['aspect']
        # The previous 224-pixel ceiling lost a 30 cm base near the configured
        # 0.6 m following distance. Search closer and farther views; measured
        # depth and physical dimensions below still decide acceptance.
        for width in (24, 28, 32, 36, 40, 48, 56, 64, 76, 88, 104, 120,
                      140, 164, 192, 224, 260, 300, 348, 404, 468, 544, 632):
            if not (.5*reference['width_m']*fx/config['max_depth'] <= width <=
                    1.8*reference['width_m']*fx/config['min_depth']):
                continue
            height = int(round(width/aspect))
            if height < 8 or width >= gray.shape[1] or height >= gray.shape[0]:
                continue
            template = cv2.resize(original, (width, height), interpolation=cv2.INTER_LINEAR)
            # Large near-body windows are searched on a pyramid, then refined
            # and scored at original resolution. This keeps full-scene fusion
            # usable on the older Kinect VM without reducing the RGB evidence.
            factor = 2 if width >= 96 else 1
            search_width, search_height = width//factor, height//factor
            search_image = half_gray if factor == 2 else gray
            search_template = cv2.resize(original, (search_width, search_height),
                                         interpolation=cv2.INTER_LINEAR)
            score = cv2.matchTemplate(search_image, search_template, cv2.TM_CCOEFF_NORMED)
            # Keep multiple local maxima. A second real matching robot must
            # remain a competing identity rather than silently pick the best.
            for unused in range(4):
                _, peak, _, position = cv2.minMaxLoc(score)
                minimum = reference.get('min_correlation', .78)
                if peak < minimum-(.10 if factor == 2 else 0.):
                    break
                x, y = position
                score[max(0,y-search_height//2):min(score.shape[0],y+search_height//2+1),
                      max(0,x-search_width//2):min(score.shape[1],x+search_width//2+1)] = -1
                if factor == 2:
                    x, y = x*factor, y*factor
                    left, top = max(0,x-3), max(0,y-3)
                    right, bottom = min(gray.shape[1],x+width+3), min(gray.shape[0],y+height+3)
                    region = gray[top:bottom,left:right]
                    if region.shape[0] < height or region.shape[1] < width:
                        continue
                    refined = cv2.matchTemplate(region, template, cv2.TM_CCOEFF_NORMED)
                    _, peak, _, position = cv2.minMaxLoc(refined)
                    x, y = left+position[0], top+position[1]
                    if peak < minimum:
                        continue
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
                # A matching RGB foreground with depth leaking through black
                # surfaces must not borrow a background plane. At that range
                # the bottom of the enrolled base would project below the floor.
                bottom = np.array(((x+width/2.-cx)*measured/fx,
                                   (y+height-1-cy)*measured/fy, measured))
                bottom_height = float(np.dot(bottom, plane[0])+plane[1])
                if not -.03 <= bottom_height <= .12:
                    continue
                proposals.append(dict(bbox=(x,y,width,height), depth=measured,
                    point=((x+width/2.-cx)*measured/fx,(y+height/2.-cy)*measured/fy,measured),
                    identity_score=1.-peak, identity_verified=peak >= reference.get('verified_correlation', .9),
                    feature=dict(kind='rgb_template_v1', label=reference['label']),
                    valid_ratio=float(same_depth.mean()), _correlation=peak,
                    _size_error=abs(math.log((width*measured/fx)/reference['width_m']))))
    return suppress_templates(proposals, mask)


def suppress_templates(proposals, mask):
    """Rank whole metric-consistent bodies before overlapping local RGB peaks.

    Correlation alone can put a half-size dark patch ahead of the complete
    enrolled base by less than 0.001. That fragment then suppresses the full
    box, leaving another fragment as a false second target. Measured depth and
    enrolled physical width resolve this competition without changing identity
    acceptance, spatial association, or separation of independent objects.
    """
    chosen = []
    for candidate in sorted(proposals, key=lambda c: -(c['_correlation']-.1*c['_size_error'])):
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
            candidate.pop('_size_error')
            chosen.append(candidate)
            mask[y:y+h,x:x+w] = 255
    return chosen, mask
