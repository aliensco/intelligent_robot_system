"""Measured open-frame evidence for a wheeled robot, not a learned identity.

This optional verifier examines only a metric ROI above an existing base
proposal. Invalid depth is unknown, never a hole in a shelf. A low platform
with observed background below supports the proposal; a continuous tapered
solid is incompatible with the enrolled open frame. Missing platforms, boxes,
occlusion, and unobserved upper parts remain unknown.
"""
import math
import numpy as np


def upper_structure_evidence(depth, camera, plane, candidate, reference_width,
                             min_depth=.4, max_depth=4.):
    result = dict(state='unknown', reason='upper_structure_unobserved')
    if plane is None:
        return result
    fx, fy, cx, cy = camera
    distance = float(candidate['depth'])
    x, y, width, height = candidate['bbox']
    middle = (x+width/2.-cx)*distance/fx
    normal, offset = plane
    # Cover a partial base's neighbouring supports, without looking across the
    # image or inventing the unseen half of a complete robot.
    left = max(0, int(math.floor((middle-reference_width)*fx/distance+cx)))
    right = min(depth.shape[1], int(math.ceil((middle+reference_width)*fx/distance+cx))+1)
    if right <= left or normal[1] >= -.5:
        return result
    # Heights are evaluated on the candidate's measured-depth plane. Background
    # pixels therefore remain in the same projected hole, rather than jumping
    # into another height bin when their actual range is farther away.
    high, low = .38, .10
    rows_at_height = [((h-offset-normal[0]*xx-normal[2]*distance)*fy/
                      (normal[1]*distance)+cy)
                     for h in (high, low) for xx in
                     (middle-reference_width, middle+reference_width)]
    top = max(0, int(math.floor(min(rows_at_height)))-1)
    bottom = min(depth.shape[0], int(math.ceil(max(rows_at_height)))+2)
    if bottom <= top:
        return result
    stride = max(1, int(math.ceil(reference_width*fx/distance/180.)))
    # Sample original measurements, never interpolated depths. Align candidate
    # ROIs to the same image lattice so duplicate proposals can still prove
    # that they share the very same observed platform pixels.
    top, left = top//stride*stride, left//stride*stride
    vv, uu = np.mgrid[top:bottom:stride, left:right:stride]
    patch = depth[top:bottom:stride, left:right:stride]
    finite = np.isfinite(patch)
    measured = np.where(finite, patch, 0.)
    # A finite sensor value outside the configured observable range is unknown,
    # not measured open space behind a platform.
    finite &= (measured >= min_depth) & (measured <= max_depth)
    xx = (uu-cx)*distance/fx
    yy = (vv-cy)*distance/fy
    heights = normal[0]*xx+normal[1]*yy+normal[2]*distance+offset
    nearby = finite & (measured >= distance-.10) & (measured <= distance+.20)
    behind = finite & (measured > distance+.25)
    rows = []
    for start in np.arange(.10, .351, .025):
        band = (heights >= start) & (heights < start+.025)
        support = band & nearby
        values = xx[support]
        row = dict(height=float(start), samples=int(len(values)))
        if len(values) >= 8:
            a, b = np.percentile(values, [5., 95.])
            region = band & (xx >= a) & (xx <= b)
            count = max(1, int(region.sum()))
            row.update(left=float(a), right=float(b), width=float(b-a),
                       center=float((a+b)/2.),
                       fill=float((nearby & region).sum())/count,
                       background=float((behind & region).sum())/count)
        rows.append(row)
    result['bands'] = rows
    platforms = []
    for row in rows:
        if (row['height'] < .20 or 'width' not in row or
                row['width'] < .35*reference_width or row['width'] > 1.5*reference_width or
                row['fill'] < .5):
            continue
        a, b = row['left'], row['right']
        base_left, base_right = (x-cx)*distance/fx, (x+width-cx)*distance/fx
        overlap = max(0., min(b, base_right)-max(a, base_left))
        if (overlap/max(1e-6, min(b-a, base_right-base_left)) < .35 or
                abs(row['center']-middle) > min(.12, .4*reference_width)):
            # A neighbouring robot's shelf cannot repair this base proposal.
            continue
        gap = ((heights >= row['height']-.08) & (heights < row['height']-.02) &
               (xx >= a) & (xx <= b))
        count = int(gap.sum())
        # A clipped ROI or a patch of NaNs is not evidence of open space.
        gap_ratio = float((gap & behind).sum())/max(1, count)
        if count < 12 or gap_ratio < .60:
            continue
        platform_region = ((heights >= row['height']) &
                           (heights < row['height']+.025) & (xx >= a) & (xx <= b))
        platform = finite & (np.abs(measured-distance) <= .10) & platform_region
        if int(platform.sum()) < 12:
            continue
        observed_x = (uu[platform]-cx)*measured[platform]/fx
        ends = np.percentile(observed_x, [5., 95.])
        # A wide farther board plus a few near pixels must not borrow its
        # apparent span/fill. The strict measured-depth support itself has to
        # form the platform, not just leave twelve samples after filtering.
        if (ends[1]-ends[0] < .35*reference_width or
                float(platform.sum())/max(1, int(platform_region.sum())) < .5):
            continue
        observed_center = float((ends[0]+ends[1])/2.)
        maximum_shift = min(.12, .4*reference_width)
        if (abs(observed_center-middle) > maximum_shift or
                abs(observed_center-candidate['point'][0]) > maximum_shift):
            continue
        platforms.append((gap_ratio*row['fill']*row['width'], row, platform, gap_ratio))
    if platforms:
        _, row, support, gap_ratio = max(platforms, key=lambda item: item[0])
        points_x = ((uu[support]-cx)*measured[support]/fx)
        point_left, point_right = np.percentile(points_x, [5., 95.])
        us, vs = uu[support], vv[support]
        result.update(state='supported', reason='measured_platform_and_open_gap',
                      platform_height_m=row['height'], gap_background_ratio=gap_ratio,
                      platform_depth_m=float(np.median(measured[support])),
                      measured_center_x=float((point_left+point_right)/2.),
                      support_bbox=[int(us.min()), int(vs.min()),
                                    int(us.max()-us.min()+1), int(vs.max()-vs.min()+1)],
                      support_samples=int(support.sum()),
                      # Internal measured-pixel identity; stripped by filter.
                      _support_pixels=(vs.astype(np.int64)*depth.shape[1]+us).tolist())
        return result
    # Reject positive evidence of a continuous taper, not absence of a shelf.
    # A constant-width carried box or an isolated chair leg is not a taper.
    solid = rows[:7]  # .10 through .25 inclusive; avoid floating endpoint drift.
    if (len(solid) >= 7 and all('width' in row and row['fill'] >= .5 and
                               row['width'] >= .025 for row in solid)):
        widths = np.array([row['width'] for row in solid])
        centers = np.array([row['center'] for row in solid])
        reduction = widths[0]-widths[-1]
        narrowing = int((np.diff(widths) <= .006).sum())
        if (reduction >= .025 and widths[-1]/widths[0] <= .75 and
                narrowing >= len(widths)-2 and np.ptp(centers) <= .04):
            result.update(state='incompatible', reason='measured_continuous_taper',
                          width_reduction_m=float(reduction),
                          upper_lower_width_ratio=float(widths[-1]/widths[0]),
                          center_span_m=float(np.ptp(centers)))
    return result


def filter_upper_structure(candidates, depth, camera, plane, reference_width,
                           diagnostics=None, min_depth=.4, max_depth=4.):
    """Verify proposals and merge only their identical measured platforms.

    Supported structure is category evidence, so it never promotes appearance
    verification or widens the tracker's identity association. The measured
    platform centre can correct a partial RGB base's horizontal position.
    """
    accepted, rejected = [], []
    for proposal in candidates:
        candidate = dict(proposal)
        evidence = upper_structure_evidence(depth, camera, plane, candidate, reference_width,
                                            min_depth, max_depth)
        pixels = evidence.pop('_support_pixels', [])
        candidate['upper_structure'] = evidence
        if evidence['state'] == 'incompatible':
            rejected.append(candidate)
            continue
        if evidence['state'] == 'supported':
            original_point = candidate['point']
            candidate['base_point'] = list(original_point)
            candidate['point'] = (evidence['measured_center_x'], original_point[1], original_point[2])
        candidate['_platform_pixels'] = set(pixels)
        accepted.append(candidate)
    groups = []
    for candidate in accepted:
        group = None
        for existing in groups:
            compatible = True
            for other in existing:
                a, b = candidate['_platform_pixels'], other['_platform_pixels']
                shared = len(a & b)
                delta = np.asarray(candidate['point'])-np.asarray(other['point'])
                base_delta = (np.asarray(candidate.get('base_point', candidate['point']))-
                              np.asarray(other.get('base_point', other['point'])))
                if (not a or not b or shared < 12 or
                        shared/float(min(len(a), len(b))) <= .65 or
                        abs(candidate['depth']-other['depth']) > .10 or
                        float(np.linalg.norm(delta)) > .12 or
                        float(np.linalg.norm(base_delta)) > .12):
                    compatible = False
                    break
            if compatible:
                group = existing
                break
        if group is None:
            groups.append([candidate])
        else:
            group.append(candidate)
    result = []
    for group in groups:
        chosen = min(group, key=lambda c: c.get('identity_score', 1.))
        chosen.pop('_platform_pixels')
        if len(group) > 1:
            chosen['upper_structure']['merged_proposals'] = len(group)
            chosen['identity_verified'] = any(c['identity_verified'] for c in group)
        result.append(chosen)
    if diagnostics is not None:
        diagnostics['rejected_candidates'] = rejected
    return result
