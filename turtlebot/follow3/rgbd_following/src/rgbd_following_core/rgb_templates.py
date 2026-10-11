"""Fixed RGB templates backed by measured body-depth pixels, no online learning."""
import cv2
import math
import numpy as np
from .body import floor_plane


def body_support_depths(depth, camera, plane, config):
    """All measured depths which could support any RGB base proposal.

    This is a necessary-condition accelerator, not a foreground detector.
    Sample every pixel: striding here could discard a thin legitimate surface.
    """
    fx, fy, cx, cy = camera
    finite = np.isfinite(depth)
    measured = np.where(finite, depth, 0.)
    valid = finite & (measured >= config['min_depth']) & (measured <= config['max_depth'])
    column_factor = (np.arange(depth.shape[1])-cx)/fx*plane[0][0]
    row_factor = (np.arange(depth.shape[0])-cy)/fy*plane[0][1]
    heights = (column_factor[None,:]+row_factor[:,None]+plane[0][2])*measured+plane[1]
    # Tiny outward epsilon keeps this algebraically rearranged, faster height
    # projection conservative at the exact original 3.5/20 cm boundaries.
    return np.sort(measured[valid & (heights > .035-1e-7) & (heights < .20+1e-7)])


def scale_has_depth_support(support_depths, width, reference_width, ratio, fx, config):
    """Skip a scale only when its original depth test cannot possibly pass.

    A successful proposal needs min_depth_samples within 0.10 m of its median.
    The permitted metric width bounds that median; extending the whole interval
    by the same 0.10 m therefore preserves even split samples whose median lies
    between their measured values. No position or highest-scoring target is used.
    """
    low = max(config['min_depth'], ratio[0]*reference_width*fx/width)
    high = min(config['max_depth'], ratio[1]*reference_width*fx/width)
    if low > high:
        return False
    left = np.searchsorted(support_depths, low-.100001, side='left')
    right = np.searchsorted(support_depths, high+.100001, side='right')
    return right-left >= config['min_depth_samples']


def detect_templates(rgb, depth, camera, config, plane=False):
    templates = config.get('rgb_templates', [])
    mask = np.zeros(depth.shape, np.uint8)
    if not templates:
        return [], mask
    fx, fy, cx, cy = camera
    if plane is False:
        from .body import floor_plane_for_depth
        plane = floor_plane_for_depth(depth, camera, config)
    if plane is None:
        return [], mask
    support_depths = body_support_depths(depth, camera, plane, config)
    if len(support_depths) < config['min_depth_samples']:
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
        ratio = reference.get('metric_width_ratio', (.5, 1.8))
        # The previous 224-pixel ceiling lost a 30 cm base near the configured
        # 0.6 m following distance. Search closer and farther views; measured
        # depth and physical dimensions below still decide acceptance.
        for width in (24, 28, 32, 36, 40, 48, 56, 64, 76, 88, 104, 120,
                      140, 164, 192, 224, 260, 300, 348, 404, 468, 544, 632):
            if not scale_has_depth_support(support_depths, width, reference['width_m'],
                                           ratio, fx, config):
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
                if not ratio[0]*reference['width_m'] <= width*measured/fx <= ratio[1]*reference['width_m']:
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
                    _depth_support=same_depth,
                    _size_error=abs(math.log((width*measured/fx)/reference['width_m']))))
    return suppress_templates(proposals, mask, max(30, config['min_depth_samples']),
                              surface_context=(depth, camera, plane, config))


def shared_template_support(first, second, min_shared_samples=30):
    """Prove overlap using the proposals' actual measured base-depth pixels."""
    first_support = first.get('_depth_support')
    second_support = second.get('_depth_support')
    if first_support is None or second_support is None:
        return False
    if abs(first['depth']-second['depth']) > .10:
        return False
    delta = np.asarray(first['point'])-np.asarray(second['point'])
    if not np.isfinite(delta).all() or float(np.linalg.norm(delta)) > .12:
        return False
    x,y,w,h = first['bbox']
    a,b,c,d = second['bbox']
    left,top,right,bottom = max(x,a),max(y,b),min(x+w,a+c),min(y+h,b+d)
    if left >= right or top >= bottom:
        return False
    if first_support.shape != (h,w) or second_support.shape != (d,c):
        return False
    overlap = (first_support[top-y:bottom-y,left-x:right-x] &
               second_support[top-b:bottom-b,left-a:right-a])
    count = int(overlap.sum())
    smaller = min(int(first_support.sum()),int(second_support.sum()))
    return count >= min_shared_samples and smaller > 0 and count/float(smaller) > .65


def template_box_overlap(first, second):
    x,y,w,h = first['bbox']
    a,b,c,d = second['bbox']
    intersection = max(0,min(x+w,a+c)-max(x,a))*max(0,min(y+h,b+d)-max(y,b))
    return abs(first['depth']-second['depth']) < .15 and intersection/float(min(w*h,c*d)) > .6


def connected_template_support(first, second, depth, camera, plane, config,
                               min_samples=30):
    """Associate vertically displaced RGB peaks on one measured base surface.

    Unlike shared pixels, a component can span two overlapping partial windows.
    It may not cross a depth hole, the ground, or a background depth step. The
    3 cm adjacent-sample tolerance allows Kinect surface noise; it is not a
    spatial gap that can be filled. Work stays inside the two small windows.
    """
    first_support = first.get('_depth_support')
    second_support = second.get('_depth_support')
    if first_support is None or second_support is None or plane is None:
        return False
    if abs(first['depth']-second['depth']) > .10:
        return False
    delta = np.asarray(first['point'])-np.asarray(second['point'])
    if not np.isfinite(delta).all() or float(np.linalg.norm(delta)) > .12:
        return False
    x,y,w,h = first['bbox']
    a,b,c,d = second['bbox']
    if first_support.shape != (h,w) or second_support.shape != (d,c):
        return False
    horizontal = max(0,min(x+w,a+c)-max(x,a))
    if min(w,c) <= 0 or horizontal/float(min(w,c)) < .8:
        return False
    left,top,right,bottom = min(x,a),min(y,b),max(x+w,a+c),max(y+h,b+d)
    fx,fy,cx,cy = camera
    maximum_height = config.get('body', {}).get('max_height_m', .20)
    if (bottom-top)*max(first['depth'],second['depth'])/fy > maximum_height:
        return False
    if left < 0 or top < 0 or right > depth.shape[1] or bottom > depth.shape[0]:
        return False
    # This optional duplicate proof must not stall the camera callback on a
    # close, fragmented surface. Exceeding the bounded work leaves ambiguity.
    if (right-left)*(bottom-top) > 65536:
        return False
    patch = depth[top:bottom,left:right]
    finite = np.isfinite(patch)
    measured = np.where(finite, patch, 0.)
    vv,uu = np.mgrid[top:bottom,left:right]
    heights = ((uu-cx)*measured/fx*plane[0][0] +
               (vv-cy)*measured/fy*plane[0][1] + measured*plane[0][2]+plane[1])
    valid = (finite & (measured >= config['min_depth']) &
             (measured <= config['max_depth']) & (heights > .035) & (heights < .20) &
             (np.abs(measured-first['depth']) <= .10) &
             (np.abs(measured-second['depth']) <= .10))
    # Do not use a connecting route outside either proposal, including corners
    # of their enclosing rectangle which were never proposed as base pixels.
    window_union = np.zeros(patch.shape, bool)
    window_union[y-top:y-top+h,x-left:x-left+w] = True
    window_union[b-top:b-top+d,a-left:a-left+c] = True
    valid &= window_union
    original_counts = (int(first_support.sum()), int(second_support.sum()))
    if min(original_counts) < min_samples:
        return False
    first_mask = np.zeros(patch.shape, bool)
    second_mask = np.zeros(patch.shape, bool)
    first_mask[y-top:y-top+h,x-left:x-left+w] = first_support
    second_mask[b-top:b-top+d,a-left:a-left+c] = second_support
    if any(int((m & valid).sum())/float(n) < .8
           for m,n in zip((first_mask,second_mask),original_counts)):
        return False
    # OpenCV floating-range flood fill checks each actual 4-neighbour depth
    # edge in compiled code. Invalid samples are occupied mask cells, never
    # interpolated. Previously visited components remain blocked by the mask.
    flood_mask = np.ones((patch.shape[0]+2,patch.shape[1]+2), np.uint8)
    flood_mask[1:-1,1:-1] = np.where(valid,0,1).astype(np.uint8)
    flags = 4 | cv2.FLOODFILL_MASK_ONLY | (255 << 8)
    surface = np.ascontiguousarray(measured, dtype=np.float32)
    visits = 0
    for sy,sx in zip(*np.where(valid & first_mask)):
        if flood_mask[sy+1,sx+1] != 0:
            continue
        if visits >= 8:
            return False
        visits += 1
        cv2.floodFill(surface, flood_mask, (int(sx),int(sy)), 0., .03, .03, flags)
        component = flood_mask[1:-1,1:-1] == 255
        counts = (int((component & first_mask).sum()),
                  int((component & second_mask).sum()))
        if (min(counts) >= min_samples and
                all(n/float(total) >= .8 for n,total in zip(counts,original_counts))):
            return True
        flood_mask[flood_mask == 255] = 1
    return False


def suppress_templates(proposals, mask, min_shared_samples=30, surface_context=None):
    """Rank whole metric-consistent bodies before overlapping local RGB peaks.

    Correlation alone can put a half-size dark patch ahead of the complete
    enrolled base by less than 0.001. That fragment then suppresses the full
    box, leaving another fragment as a false second target. Measured depth and
    enrolled physical width resolve this competition without changing identity
    acceptance, spatial association, or separation of independent objects.
    """
    def measured_match(first, second):
        if shared_template_support(first, second, min_shared_samples):
            return True
        if surface_context is None:
            return False
        depth,camera,plane,config = surface_context
        return connected_template_support(first, second, depth,camera,plane,config,
                                          min_samples=min_shared_samples)

    chosen, groups, measured_groups = [], [], []
    for proposal in sorted(proposals, key=lambda c: -(c['_correlation']-.1*c['_size_error'])):
        candidate = dict(proposal)
        x,y,w,h = candidate['bbox']
        duplicate = False
        for group_index,(old,group) in enumerate(zip(chosen,groups)):
            geometric = template_box_overlap(candidate, old)
            measured = False if geometric else measured_match(candidate, old)
            # Once a group uses new measured evidence, every subsequent join
            # (including a geometric one) needs pairwise evidence. Otherwise
            # A--B shared pixels followed by B--C box overlap could hide two
            # distinct A/C surfaces. Pure original whole-box NMS is unchanged.
            eligible = geometric or measured
            if eligible and (not geometric or measured_groups[group_index]):
                eligible = all(template_box_overlap(candidate, member) or
                               measured_match(candidate, member)
                               for member in group[1:])
            if eligible:
                duplicate = True
                group.append(candidate)
                measured_groups[group_index] |= not geometric
                break
        if not duplicate:
            chosen.append(candidate)
            groups.append([candidate])
            measured_groups.append(False)
            mask[y:y+h,x:x+w] = 255
    for candidate in chosen:
        candidate.pop('_correlation')
        candidate.pop('_size_error')
        candidate.pop('_depth_support', None)
    return chosen, mask
