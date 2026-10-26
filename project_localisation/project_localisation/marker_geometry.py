"""Geometry helpers for ArUco marker pose estimation and localisation.

This module provides quaternion-based coordinate transformations, marker corner
geometry, single-frame pose estimation, multi-observation fusion, and record
updates. It is designed to be used by the target localisation ROS 2 node.
"""

import math

import cv2
import numpy as np


def rotation(q):
    """Convert a quaternion in ``[x, y, z, w]`` order to a rotation matrix.

    The quaternion is normalised before conversion.

    Args:
        q: Four-element quaternion in ``[x, y, z, w]`` order.

    Returns:
        A 3-by-3 rotation matrix.

    Raises:
        ValueError: If the quaternion has the wrong shape, contains non-finite
            values, or has a near-zero norm.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid quaternion')

    x, y, z, w = q / np.linalg.norm(q)

    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform(point, translation, quaternion):
    """Transform a 3D point into a frame described by a pose.

    Args:
        point: Three-dimensional point in the source frame.
        translation: Three-element translation of the target frame.
        quaternion: Target-frame orientation as ``[x, y, z, w]``.

    Returns:
        The point rotated by the quaternion and offset by the translation.
    """
    return rotation(quaternion) @ np.asarray(point) + np.asarray(translation)


def square_points(size):
    """Return the four 3D corner coordinates of a centred square marker.

    The corners are ordered to match the convention expected by
    ``cv2.SOLVEPNP_IPPE_SQUARE``.

    Args:
        size: Marker side length in metres.

    Returns:
        A 4-by-3 NumPy array of marker corner coordinates.

    Raises:
        ValueError: If ``size`` is not a finite positive length.
    """
    if not math.isfinite(size) or size <= 0:
        raise ValueError('marker_size must be a positive length in metres')

    h = size / 2
    return np.array([
        [-h, h, 0],
        [h, h, 0],
        [h, -h, 0],
        [-h, -h, 0],
    ], dtype=float)


def estimate(
    corners,
    size,
    K,
    D,
    width,
    height,
    min_side=25.0,
    max_range=3.0,
    max_error=2.0,
):
    """Estimate an ArUco marker's 3D pose from its image corners.

    The function filters detections near image boundaries or below the minimum
    pixel size, tries the IPPE-square and iterative PnP solvers, and selects
    the valid candidate with the lowest reprojection error.

    Args:
        corners: Detected marker corners, containing four 2D image points.
        size: Physical marker side length in metres.
        K: 3-by-3 camera intrinsic matrix.
        D: Camera distortion coefficients.
        width: Image width in pixels.
        height: Image height in pixels.
        min_side: Minimum accepted marker edge length in pixels.
        max_range: Maximum accepted camera-to-marker distance in metres.
        max_error: Maximum accepted reprojection RMS error in pixels.

    Returns:
        A dictionary containing ``xyz``, ``reprojection``, ``range_m`` and
        ``view_angle_deg`` for the best valid pose, or ``None`` if no candidate
        passes the quality checks.
    """
    p = np.asarray(corners, dtype=float).reshape(4, 2)
    if not np.isfinite(p).all():
        return None

    sides = np.linalg.norm(p - np.roll(p, 1, axis=0), axis=1)
    if (
        min(sides) < min_side
        or np.min(p) < 5
        or np.max(p[:, 0]) > width - 6
        or np.max(p[:, 1]) > height - 6
    ):
        return None

    objects = square_points(size)
    candidates = []

    try:
        ok, rotations, translations, _ = cv2.solvePnPGeneric(
            objects,
            p,
            K,
            D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if ok:
            candidates.extend(zip(rotations, translations))
    except cv2.error:
        pass

    try:
        ok, rv, tv = cv2.solvePnP(
            objects,
            p,
            K,
            D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if ok:
            candidates.append((rv, tv))
    except cv2.error:
        pass

    best = None
    for rv, tv in candidates:
        xyz = tv.reshape(3)
        distance = float(np.linalg.norm(xyz))

        if (
            not np.isfinite(xyz).all()
            or not np.isfinite(rv).all()
            or xyz[2] <= 0
        ):
            continue

        if not 0.15 <= distance <= max_range:
            continue

        R, _ = cv2.Rodrigues(rv)
        cosine = abs(float(R[:, 2] @ xyz)) / distance
        angle = math.degrees(math.acos(float(np.clip(cosine, 0, 1))))
        if angle > 60:
            continue

        projected, _ = cv2.projectPoints(objects, rv, tv, K, D)
        rms = float(np.sqrt(np.mean(
            np.sum((projected.reshape(4, 2) - p) ** 2, axis=1)
        )))

        if rms <= max_error and (
            best is None or rms < best['reprojection']
        ):
            best = dict(
                xyz=xyz.tolist(),
                reprojection=rms,
                range_m=distance,
                view_angle_deg=angle,
            )

    return best


def fuse(samples, min_n=8, min_span=1.5, min_baseline=0.15, max_scatter=0.15):
    """Fuse multiple observations of a target into one position estimate.

    A median-based residual filter removes outliers before calculating the
    centre and scatter. Confirmation depends on the number of retained
    observations, their time span, and the resulting positional scatter.

    Args:
        samples: Observation tuples of ``(time, point, camera_position)``.
        min_n: Minimum number of inlier observations required for confirmation.
        min_span: Minimum observation time span in seconds for confirmation.
        min_baseline: Minimum camera movement distance in metres (retained as
            an argument for compatibility; it is not used in the confirmation
            expression in this implementation).
        max_scatter: Maximum allowed 95th-percentile positional scatter in
            metres for confirmation.

    Returns:
        A dictionary containing the fused ``xyz`` position, observation count,
        scatter, camera baseline, time span, confirmation flag and latest
        inlier timestamp; or ``None`` if the input is invalid or empty.
    """
    if not samples:
        return None

    points = np.asarray([s[1] for s in samples], dtype=float)
    cameras = np.asarray([s[2] for s in samples], dtype=float)
    times = np.asarray([s[0] for s in samples], dtype=float)

    if (
        not np.isfinite(points).all()
        or not np.isfinite(cameras).all()
        or not np.isfinite(times).all()
        or np.any(np.diff(times) < 0)
    ):
        return None

    centre = np.median(points, axis=0)
    residual = np.linalg.norm(points - centre, axis=1)
    med = float(np.median(residual))
    mad = float(np.median(np.abs(residual - med)))
    keep = residual <= max(0.08, med + 3 * 1.4826 * mad)

    points, cameras, times = points[keep], cameras[keep], times[keep]
    if not len(points):
        return None

    centre = np.median(points, axis=0)
    scatter = float(np.quantile(np.linalg.norm(points - centre, axis=1), 0.95))
    baseline = float(np.max(np.linalg.norm(
        cameras[:, None, :] - cameras[None, :, :],
        axis=2,
    )))
    span = float(times[-1] - times[0])

    confirmed = (
        len(points) >= min_n
        and span >= min_span
        and scatter <= max_scatter
    )

    return dict(
        xyz=centre.tolist(),
        n=len(points),
        scatter=scatter,
        baseline=baseline,
        span=span,
        confirmed=bool(confirmed),
        inlier_stamp=float(times[-1]),
    )


def update_record(previous, current, stamp, tolerance=0.25):
    """Update a target record while protecting an existing confirmed position.

    A new estimate replaces the previous record unless the previous estimate
    is confirmed and the new estimate is tentative. In that case, the previous
    confirmed position is retained when the tentative estimate is within the
    specified distance tolerance.

    Args:
        previous: Previously stored record, or ``None`` if no record exists.
        current: Newly fused estimate.
        stamp: Time of the latest observation.
        tolerance: Maximum distance in metres for retaining the prior
            confirmed position when a tentative estimate is received.

    Returns:
        The updated record dictionary.
    """
    result = dict(
        current,
        last_seen=stamp,
        position_stamp=current.get('inlier_stamp', stamp),
    )

    if previous is None or not previous['confirmed'] or current['confirmed']:
        return result

    distance = float(np.linalg.norm(
        np.asarray(current['xyz']) - np.asarray(previous['xyz'])
    ))
    if distance > tolerance:
        result['reason'] = 'New observations conflict with the confirmed position'
        return result

    retained = dict(previous, last_seen=stamp)
    return retained
