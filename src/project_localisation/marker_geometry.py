import math
import numpy as np
import cv2



def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def transform(point, translation, quaternion):
    return rotation(quaternion) @ np.asarray(point) + np.asarray(translation)


def grid_world(y, x, resolution, origin, yaw):
    a, b = (x+0.5)*resolution, (y+0.5)*resolution
    c, s = math.cos(yaw), math.sin(yaw)
    return origin[0]+c*a-s*b, origin[1]+s*a+c*b


def world_grid(x, y, resolution, origin, yaw):
    a, b = x-origin[0], y-origin[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return math.floor((b*c-a*s)/resolution), math.floor((a*c+b*s)/resolution)


def square_points(size):
    if not math.isfinite(size) or size <= 0:
        raise ValueError('marker_size must be a positive length in metres')
    h = size/2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)


def estimate(corners, size, K, D, width, height,
             min_side=25.0, max_range=3.0, max_error=2.0):
    p = np.asarray(corners, dtype=float).reshape(4, 2)
    if not np.isfinite(p).all():
        return None
    sides = np.linalg.norm(p-np.roll(p, 1, axis=0), axis=1)
    if (min(sides) < min_side or np.min(p) < 5
            or np.max(p[:, 0]) > width-6 or np.max(p[:, 1]) > height-6):
        return None
    objects = square_points(size)
    candidates = []
    try:
        ok, rotations, translations, _ = cv2.solvePnPGeneric(
            objects, p, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            candidates.extend(zip(rotations, translations))
    except cv2.error:
        pass
    try:
        ok, rv, tv = cv2.solvePnP(objects, p, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            candidates.append((rv, tv))
    except cv2.error:
        pass
    best = None
    for rv, tv in candidates:
        xyz = tv.reshape(3)
        distance = float(np.linalg.norm(xyz))
        if not np.isfinite(xyz).all() or not np.isfinite(rv).all() or xyz[2] <= 0:
            continue
        if not 0.15 <= distance <= max_range:
            continue
        R, _ = cv2.Rodrigues(rv)
        cosine = abs(float(R[:, 2] @ xyz)) / distance
        angle = math.degrees(math.acos(float(np.clip(cosine, 0, 1))))
        if angle > 60:
            continue
        projected, _ = cv2.projectPoints(objects, rv, tv, K, D)
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2)-p)**2, axis=1))))
        if rms <= max_error and (best is None or rms < best['reprojection']):
            best = dict(xyz=xyz.tolist(), reprojection=rms,
                        range_m=distance, view_angle_deg=angle)
    return best


def fuse(samples, min_n=8, min_span=1.5, min_baseline=0.15, max_scatter=0.15):
    if not samples:
        return None
    points = np.asarray([s[1] for s in samples], dtype=float)
    cameras = np.asarray([s[2] for s in samples], dtype=float)
    times = np.asarray([s[0] for s in samples], dtype=float)
    if (not np.isfinite(points).all() or not np.isfinite(cameras).all()
            or not np.isfinite(times).all() or np.any(np.diff(times) < 0)):
        return None
    centre = np.median(points, axis=0)
    residual = np.linalg.norm(points-centre, axis=1)
    med = float(np.median(residual))
    mad = float(np.median(np.abs(residual-med)))
    keep = residual <= max(0.08, med+3*1.4826*mad)
    points, cameras, times = points[keep], cameras[keep], times[keep]
    if not len(points):
        return None
    centre = np.median(points, axis=0)
    scatter = float(np.quantile(np.linalg.norm(points-centre, axis=1), 0.95))
    baseline = float(np.max(np.linalg.norm(cameras[:, None, :]-cameras[None, :, :], axis=2)))
    span = float(times[-1]-times[0])
    confirmed = (len(points) >= min_n and span >= min_span
                 and baseline >= min_baseline and scatter <= max_scatter)
    return dict(xyz=centre.tolist(), n=len(points), scatter=scatter,
                baseline=baseline, span=span, confirmed=bool(confirmed),
                inlier_stamp=float(times[-1]))


def update_record(previous, current, stamp, tolerance=0.25):
    result = dict(current, last_seen=stamp,
                  position_stamp=current.get('inlier_stamp', stamp))
    if previous is None or not previous['confirmed'] or current['confirmed']:
        return result
    distance = float(np.linalg.norm(np.asarray(current['xyz'])-np.asarray(previous['xyz'])))
    if distance > tolerance:
        result['reason'] = 'New observations conflict with the confirmed position'
        return result
    retained = dict(previous, last_seen=stamp)
    return retained
