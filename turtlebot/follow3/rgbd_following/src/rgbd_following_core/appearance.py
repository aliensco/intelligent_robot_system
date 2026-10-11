"""Fixed, enrolled multiview signatures. Never learn identity from predictions."""
import math
from .depth import finite


def signature_distance(left, right):
    """Hellinger distance on normalized spatial HSV histograms, in [0, 1]."""
    if len(left) != 144 or len(right) != 144:
        raise ValueError('Expected four 12x3 HSV histograms')
    if not all(finite(v) and v >= 0 for v in list(left) + list(right)):
        raise ValueError('Invalid appearance histogram')
    a, b = sum(left), sum(right)
    if a <= 0 or b <= 0:
        raise ValueError('Empty appearance histogram')
    return math.sqrt(max(0., 1. - sum(math.sqrt(x / a * y / b) for x, y in zip(left, right))))


def match_reference(feature, references, shape_ratio=2., width_ratio=1.8):
    scores = []
    for ref in references:
        if feature.get('kind', 'color_v1') != ref.get('kind', 'color_v1'):
            continue
        shape = feature['aspect'] / ref['aspect']
        width = feature['width_m'] / ref['width_m']
        if 1. / shape_ratio <= shape <= shape_ratio and 1. / width_ratio <= width <= width_ratio:
            left, right = feature['histogram'], ref['histogram']
            if feature.get('kind') == 'depth_body_v1':
                left, right = smooth_body_brightness(left), smooth_body_brightness(right)
            score = signature_distance(left, right)
            if feature.get('kind') == 'depth_body_v1':
                height = feature['height_m'] / ref['height_m']
                if not 1. / width_ratio <= height <= width_ratio:
                    continue
                outline = math.sqrt(sum((x-y)**2 for x,y in zip(feature['silhouette'], ref['silhouette'])) / 64.)
                score = max(score, .6 * outline)
            scores.append(score)
    return min(scores) if scores else 1.


def smooth_body_brightness(histogram):
    """Tolerate adjacent exposure bins, preserving quadrant and color identity.

    Body histograms put gray pixels in twelve brightness bins. A small exposure
    change used to make adjacent bins disjoint under Hellinger distance. Apply
    the SAME mass-preserving kernel to saved and live descriptors, without
    widening geometry gates, learning predictions, or mixing hue with gray.
    """
    if len(histogram) != 144:
        raise ValueError('Expected body histogram with four 36-bin quadrants')
    result = list(histogram)
    for unused in range(3):
        previous = result[:]
        for start in range(0, 144, 36):
            for i in range(12):
                result[start+i] = 0.
            for i in range(12):
                value = previous[start+i]
                result[start+max(0, i-1)] += .25*value
                result[start+i] += .5*value
                result[start+min(11, i+1)] += .25*value
    return result
