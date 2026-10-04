import math
import numpy as np
import cv2


#geometry
def rotation(q):
    x,y,z,w = np.asarray(q, dtype=float)
    n = math.sqrt(x*x + y*y + z*z + w*w)

    if n < 1e-8:
        raise ValueError('Zero Quaternion')

    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([

        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-y*w), 2*(y*z+w*x), 1-2*(y*y+x*x)]

    ])

def transform(point, translation, quaternion):
    return rotation(quaternion) @ point + translation

def grid_world(y, x, resolution, origin, yaw):
    a = (x+0.5)*resolution
    b = (y+0.5)*resolution
    c = math.cos(yaw)
    s = math.sin(yaw)
    return origin[0] + c*a - b*s, origin[1] + s*a + b*c

def world_grid(x, y, resolution,origin, yaw):
    a = x - origin[0]
    b = y - origin[1]
    c = math.cos(yaw)
    s = math.sin(yaw)
    return math.floor((b * c - a * s) / resolution), math.floor((a * c + b * s) / resolution)



def square_points(size):
    if (not math.isfinite(size)) or (size <= 0):
        raise ValueError('Marker_size should be positive and in meters')
    h = size/2
    return np.array([
        [-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]
    ],
    dtype = np.float64
    )

def estimate(corners, size, K, D, width, height,
             min_side=25.0, max_range=3.0, max_error=2.0):
    p = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(p).all():
        return None
    sides = np.linalg.norm(p - np.roll(p, 1, axis=0), axis=1)
    if min(sides) < min_side or np.min(p) < 5 or np.max(p[:, 0]) > width - 6 or np.max(p[:, 1]) > height - 6:
        return None
    objects = square_points(size)
    ok, rotations, translations, _ = cv2.solvePnPGeneric(objects, p, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = list(zip(rotations, translations)) if ok else []
    iterative_ok, rv, tv = cv2.solvePnP(objects, p, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    if iterative_ok:
        candidates.append((rv, tv))
    best = None
    for rv, tv in candidates:
        xyz = tv.reshape(3)
        distance = float(np.linalg.norm(xyz))
        if not np.isfinite(xyz).all() or xyz[2] <= 0:
            continue
        if not 0.15 <= distance <= max_range:
            continue
        R, _ = cv2.Rodrigues(rv)
        if abs(float(R[:, 2] @ xyz)) / distance < math.cos(math.radians(60)):
            continue
        projected, _ = cv2.projectPoints(objects, rv, tv, K, D)
        rms = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(4, 2) - p) ** 2, axis=1))))

        if rms <= max_error and (best is None or rms < best['reprojection']):
            best = dict(xyz=xyz.tolist(), reprojection=rms)
    return best


#fusion

def fuse(
        samples, min_n=8, min_span=1.5, min_baseline=0.15, max_scatter=0.15
):
    if not samples:
        return None
    points = np.array([s[1] for s in samples])
    centre = np.median(points, axis=0)
    residual = np.linalg.norm(points - centre, axis=1)
    med = float(np.median(residual))

    mad = float(np.median(np.abs(residual - med)))

    keep = residual <= max(0.08, med + 3 * 1.4826 * mad)
    good = [s for s, ok in zip(samples, keep) if ok]
    points = points[keep]
    centre = np.median(points, axis=0)
    scatter = float(np.quantile(np.linalg.norm(points-centre, axis=1), 0.95))
    cameras = np.array([s[2] for s in good])
    baseline = float(np.max(np.linalg.norm(cameras[:, None, :] - cameras[None, :, :], axis=2)))
    span = good[-1][0] - good[0][0]
    confirmed = (len(good) >= min_n
                 and span >= min_span
                 and baseline >= min_baseline
                 and scatter <= max_scatter)
    return dict(xyz=centre.tolist(), n=len(good),
                scatter=scatter, baseline=baseline,
                confirmed=bool(confirmed))

def update_record(previous, current, stamp, tolerance=0.25):
    result = dict(current, last_seen=stamp, position_stamp=stamp)
    if previous is None or not previous['confirmed'] or current['confirmed']:
        return result
    distance = float(np.linalg.norm(
        np.asarray(current['xyz']) - np.asarray(previous['xyz'])))

    if distance > tolerance:
        result['confirmed'] = False
        result['reason'] = 'New observations conflict with the confirmed position'
        return result

    retained = dict(previous)
    retained['last_seen'] = stamp
    return retained



































