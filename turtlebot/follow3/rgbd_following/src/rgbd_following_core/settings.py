"""Reject invalid configuration before hardware outputs are created."""
from .depth import finite
from .tracking import Tracker
from .appearance import signature_distance


def positive(config, keys):
    for key in keys:
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not finite(value) or value <= 0:
            raise ValueError('Expected positive finite setting: ' + key)


def validate(config, execute=False):
    positive(config, ('sync_slop', 'target_timeout', 'mission_timeout', 'tf_timeout',
                      'grid_timeout', 'follow_distance', 'minimum_distance', 'footprint_radius',
                      'max_linear', 'max_angular', 'braking_acceleration', 'reaction_time',
                      'finish_tolerance', 'finish_hold', 'finish_motion_tolerance', 'lost_failure_seconds',
                      'odom_timeout', 'stopped_linear', 'stopped_angular', 'angular_braking_acceleration'))
    visibility = config['visibility']
    positive(visibility, ('horizon', 'margin', 'camera_timeout'))
    if not 0 < visibility['margin'] < .45 or visibility['horizon'] > 3:
        raise ValueError('Invalid visibility horizon/margin')
    identity = config['perception']['identity']
    positive(identity, ('max_distance', 'reacquire_distance', 'shape_ratio', 'width_ratio'))
    if not 0 < identity['reacquire_distance'] <= identity['max_distance'] < 1:
        raise ValueError('Invalid appearance thresholds')
    if min(identity['shape_ratio'], identity['width_ratio']) <= 1:
        raise ValueError('Identity shape ratios must exceed one')
    for ref in identity['references']:
        positive(ref, ('aspect', 'width_m'))
        signature_distance(ref['histogram'], ref['histogram'])
        kind = ref.get('kind', 'color_v1')
        if kind not in ('color_v1', 'depth_body_v1'):
            raise ValueError('Unknown identity feature kind')
        if kind == 'depth_body_v1':
            positive(ref, ('height_m',))
            if len(ref.get('silhouette', [])) != 64 or not all(finite(v) and 0 <= v <= 1 for v in ref['silhouette']):
                raise ValueError('Invalid body silhouette')
    # Target depth is a surface point, not the other vehicle's centre.
    if not config['follow_distance'] > config['minimum_distance'] > config['footprint_radius']:
        raise ValueError('Require follow_distance > minimum_distance > footprint radius')
    if config['sync_slop'] >= config['target_timeout']:
        raise ValueError('Sync slop must be less than target timeout')
    if config['max_linear'] > .3 or config['max_angular'] > .8:
        raise ValueError('Competition speed limit exceeded')
    p = config['perception']
    for key in ('detour_max_extra_distance', 'navigation_retry_delay', 'detour_goal_hold'):
        if key in config:
            positive(config, (key,))
    if 'standoff_min_distance' in config:
        if not config['minimum_distance'] < config['standoff_min_distance'] <= config['follow_distance']:
            raise ValueError('Standoff minimum must exceed stop distance and not exceed desired gap')
    templates = p.get('rgb_templates', [])
    if not isinstance(templates, list) or len(templates) > 8:
        raise ValueError('Expected up to eight fixed RGB templates')
    for template in templates:
        pixels = template.get('pixels', [])
        if (not 8 <= len(pixels) <= 64 or not isinstance(pixels[0], list) or
                not 8 <= len(pixels[0]) <= 64 or
                any(not isinstance(row, list) or len(row) != len(pixels[0]) or
                    any(not isinstance(v, int) or not 0 <= v <= 255 for v in row) for row in pixels)):
            raise ValueError('Invalid fixed RGB template pixels')
        if (not template.get('label') or
                not .5 <= template.get('aspect', 0) <= 6 or
                not 0 < template.get('width_m', 0) <= .8 or
                not .7 <= template.get('min_correlation', .78) <= template.get('verified_correlation', .9) <= 1):
            raise ValueError('Invalid fixed RGB template geometry or correlation')
    mode = p.get('proposal_mode', 'color')
    if mode not in ('color', 'depth_body'):
        raise ValueError('Unknown proposal mode')
    expected = 'depth_body_v1' if mode == 'depth_body' else 'color_v1'
    if any(ref.get('kind', 'color_v1') != expected for ref in identity['references']):
        raise ValueError('Identity references do not match proposal mode; enroll new views')
    if mode == 'depth_body':
        body = p['body']
        if 'max_support_height_m' in body:
            positive(body, ('max_support_height_m',))
            if not .035 < body['max_support_height_m'] <= .8:
                raise ValueError('Invalid ground-relative recognition support height')
        positive(body, ('min_width_m', 'max_width_m', 'min_height_m', 'max_height_m'))
        if not (body['min_width_m'] < body['max_width_m'] <= .8 and
                body['min_height_m'] < body['max_height_m'] <= .8):
            raise ValueError('Invalid body dimensions')
    positive(p, ('min_area_ratio', 'max_area_ratio', 'min_depth', 'max_depth',
                 'min_depth_samples', 'min_valid_ratio', 'max_depth_spread', 'depth_cluster_band',
                 'min_width_m', 'max_width_m', 'cloud_stride', 'min_cloud_points', 'min_cloud_ratio'))
    if not (p['min_area_ratio'] < p['max_area_ratio'] <= 1 and p['min_depth'] < p['max_depth'] and
            p['min_width_m'] < p['max_width_m'] and p['min_valid_ratio'] <= 1 and p['min_cloud_ratio'] <= 1):
        raise ValueError('Invalid perception ranges')
    for key in ('cloud_stride', 'min_cloud_points', 'min_depth_samples'):
        if not isinstance(p[key], int):
            raise ValueError('Expected integer: ' + key)
    p.setdefault('clearing_max_depth', p['max_depth'])
    p.setdefault('clearing_stride', p['cloud_stride'])
    positive(p, ('clearing_max_depth', 'clearing_stride'))
    if p['clearing_max_depth'] < p['max_depth']:
        raise ValueError('Clearing range must include the obstacle range')
    if not isinstance(p['clearing_stride'], int) or p['clearing_stride'] > p['cloud_stride']:
        raise ValueError('Clearing stride must be an integer no coarser than obstacle stride')
    if not p['hsv_bands']:
        raise ValueError('At least one calibrated HSV band is required')
    for band in p['hsv_bands']:
        if len(band['lower']) != 3 or len(band['upper']) != 3:
            raise ValueError('HSV bands require three channels')
        for lo, hi, limit in zip(band['lower'], band['upper'], (179, 255, 255)):
            if not isinstance(lo, int) or not isinstance(hi, int) or not 0 <= lo <= hi <= limit:
                raise ValueError('Invalid HSV band; split wrap-around hue into two bands')
    Tracker(**config['tracker'])
    if config['tracker']['timeout'] > config['target_timeout']:
        raise ValueError('Tracker timeout must not exceed target timeout')
    poly = config['finish_polygon']
    if poly:
        if len(poly) < 3 or any(len(p) != 2 or not all(finite(v) for v in p) for p in poly):
            raise ValueError('Finish polygon requires finite 2D vertices')
        turns = []
        for i, (a, b) in enumerate(zip(poly, poly[1:] + poly[:1])):
            for j, point in enumerate(poly):
                if j not in (i, (i + 1) % len(poly)):
                    turns.append((b[0]-a[0])*(point[1]-a[1]) - (b[1]-a[1])*(point[0]-a[0]))
        if not (all(t > 0 for t in turns) or all(t < 0 for t in turns)):
            raise ValueError('Finish polygon must be convex, ordered and nondegenerate')
    if execute:
        if not identity['references']:
            raise ValueError('Hardware execution requires enrolled identity references')
        if not all(config[k] is True for k in ('hardware_calibrated', 'identity_calibrated', 'depth_coverage_calibrated')):
            raise ValueError('Record identity, footprint, braking and depth coverage calibration before execute')
        if not poly:
            raise ValueError('Hardware execution requires a measured finish_polygon')
    return config
