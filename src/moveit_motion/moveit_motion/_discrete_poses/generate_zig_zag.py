#!/usr/bin/env python3
"""
zigzag_path.py
~~~~~~~~~~~~~~
Generate, plot and save/read zig‑zag trajectories with constant orientation.
"""

from __future__ import annotations
import csv
from pathlib import Path

import matplotlib.pyplot as plt      # install matplotlib if absent
import numpy as np

# -------------------------------------------------------------------- constants
_HEADER = ["X", "Y", "Z", "x", "y", "z", "w"]


# ---------------------------------------------------------------- generate path
def generate_zigzag(p_start, p_second, p_end, M: int, N: int,
                    *, return_vertices: bool = False):
    """
    Create a zig‑zag trajectory with a fixed quaternion orientation.

    Parameters
    ----------
    p_start, p_second, p_end : sequence of 7 floats
        Cartesian + quaternion poses used exactly as in your specification.
    M : int
        Number of vertices (direction‑change points), inclusive of the first
        and last.
    N : int
        Number of samples taken along **each** straight segment, inclusive of
        both segment end‑points.  Must be ≥ 2.
    return_vertices : bool, optional
        If True, the function returns a tuple ``(waypoints, vertices)``;
        otherwise it returns only ``waypoints``.

    Returns
    -------
    waypoints : ndarray, shape(((M‑1)*N) + 1, 7)
    vertices  : ndarray, shape(M, 3)              (only if return_vertices)
    """
    if M < 2:
        raise ValueError("M must be at least 2 (start and end vertex).")
    if N < 2:
        raise ValueError("N must be at least 2 (per segment).")

    p_start, p_second, p_end = map(
        lambda p: np.asarray(p, dtype=float), (p_start, p_second, p_end)
    )
    quat = p_start[3:]              # fixed orientation
    z_val = p_start[2]              # planar motion => constant Z

    # 1 ── build the M vertices ------------------------------------------------
    y_vertices = np.linspace(p_start[1], p_end[1], M)
    x_first, x_second = p_start[0], p_second[0]

    vertices = []
    for i in range(M):
        if i == M - 1:              # force the very last vertex to p_end.xx
            x_i = p_end[0]
        else:                       # alternate sideways along X
            x_i = x_first if i % 2 == 0 else x_second
        vertices.append([x_i, y_vertices[i], z_val])
    vertices = np.asarray(vertices)

    # 2 ── interpolate N samples on each leg ----------------------------------
    waypoints = []
    for i in range(M - 1):
        pA, pB = vertices[i], vertices[i + 1]
        for j in range(N):          # includes both ends
            t = j / (N - 1)
            pos = (1.0 - t) * pA + t * pB
            waypoints.append(np.concatenate([pos, quat]))

    # ensure the exact final position is present
    waypoints.append(np.concatenate([vertices[-1], quat]))
    waypoints = np.asarray(waypoints)

    return (waypoints, vertices) if return_vertices else waypoints


# -------------------------------------------------------------------- plotting
def plot_zig_zag(vertices, waypoints, *, label_vertices: bool = True, ax=None):
    """
    Plot the XY projection of the path, marking and optionally labelling
    the M vertices.

    Parameters
    ----------
    vertices   : (M, 3) array-like
    waypoints  : (N_pts, 7) array-like
    label_vertices : bool, default True
    ax         : matplotlib.axes.Axes, optional
        Provide an Axes to draw into, or let the function create its own.
    """
    vertices = np.asarray(vertices, dtype=float)
    waypoints = np.asarray(waypoints, dtype=float)

    if ax is None:
        _, ax = plt.subplots(figsize=(6, 6))

    ax.plot(waypoints[:, 0], waypoints[:, 1], lw=1.4)
    ax.scatter(vertices[:, 0], vertices[:, 1], s=45)

    if label_vertices:
        for k, (xv, yv) in enumerate(vertices[:, :2], start=1):
            ax.annotate(str(k), (xv, yv),
                        textcoords="offset points", xytext=(5, 5),
                        fontsize=9)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Zig‑zag path with vertices")
    ax.axis("equal")
    ax.grid(True)


# ----------------------------------------------------------------------- I/O ‑‑
def export_waypoints_csv(path, waypoints):
    """
    Write *waypoints* to *path* with header ``X, Y, Z, x, y, z, w``.

    Parameters
    ----------
    path : str or pathlib.Path
    waypoints : (N_pts, 7) array-like
    """
    path = Path(path)
    wp = np.asarray(waypoints, dtype=float)
    if wp.shape[1] != 7:
        raise ValueError("waypoints must have shape (*, 7).")

    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(_HEADER)
        writer.writerows(wp)


def read_waypoints_csv(path):
    """
    Read a CSV file written by :func:`export_waypoints_csv`.

    Returns
    -------
    np.ndarray, shape(N_pts, 7)
    """
    path = Path(path)
    with path.open(newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        if [h.strip() for h in header] != _HEADER:
            raise ValueError(
                f"Unexpected header {header} – expected {_HEADER!r}"
            )
        data = [[float(x) for x in row] for row in reader]

    return np.asarray(data, dtype=float)


# -------------------------------------------------------------------- example
if __name__ == "__main__":
    # ---- input poses --------------------------------------------------------
    P1 = [1.3661,  0.27332,  -0.09536,  0.29518, 0.79266, 0.19416, 0.49685]
    P2 = [1.3304,  0.22664,  -0.09536,  0.29526, 0.79265, 0.19407, 0.49685]
    PM = [1.3808, -0.021355, -0.09536,  0.29527, 0.79267, 0.19406, 0.49683]

    M, N = 20, 20                              # 10 vertices, 20 samples/leg

    # ---- generate ----------------------------------------------------------
    waypoints, vertices = generate_zigzag(
        P1, P2, PM, M, N, return_vertices=True
    )
    print(f"Generated {len(waypoints)} way‑points.")

    # ---- plot --------------------------------------------------------------
    plot_zig_zag(vertices, waypoints)
    plt.show()

    # ---- export & reload ---------------------------------------------------
    csv_file = "zigzag_path.csv"
    export_waypoints_csv(csv_file, waypoints)
    restored = read_waypoints_csv(csv_file)

    assert np.allclose(restored, waypoints)
    print(f"CSV round‑trip OK – {restored.shape[0]} points written & read.")
