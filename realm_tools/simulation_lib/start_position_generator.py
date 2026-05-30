"""
start_position_generator.py

Generates a uniform grid of valid robot start positions for a maze
defined in a REALM environment XML file.

Algorithm
---------
1. Parse all wall segments from the XML.
2. Build the outer boundary polygon as the convex hull of the boundary-typed
   wall endpoints (falls back to all walls for untyped XMLs).
3. Lay a uniform grid over the boundary bounding box.
4. Keep every point that is (a) inside the boundary polygon and (b) at least
   wall_clearance metres from every wall segment (boundary and obstacle).
5. Save to CSV  (x, y, theta).

Parameters
----------
spacing       Distance between adjacent grid points (metres).
              Smaller values = denser coverage, more habituation positions.
wall_clearance Minimum distance from any wall surface to a grid point (metres).
              Acts as a safety buffer — points closer than this to any wall
              are excluded.

Usage — from Python
-------------------
    from realm_tools.simulation_lib.start_position_generator import generate_grid

    df = generate_grid(
        'simulation/worlds/environments/vpce/LMO8.xml',
        spacing=0.1,
        wall_clearance=0.2,
        output='simulation/worlds/environments/vpce/LMO8_positions.csv',
        plot=True,
    )

Usage — from the command line
------------------------------
    python -m realm_tools.simulation_lib.start_position_generator \\
        simulation/worlds/environments/vpce/LMO8.xml \\
        --spacing 0.1 --clearance 0.2 --theta 0.0 \\
        --output simulation/worlds/environments/vpce/LMO8_positions.csv \\
        --plot
"""

import argparse
import os
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd
from shapely.geometry import LineString, MultiPoint, Point

from realm_tools.simulation_lib.environment_parser import parse_all_walls

# Robot physical constants (from HamBot spec)
ROBOT_RADIUS   = 0.3086   # metres
MIN_LIDAR_DIST = 0.50     # metres — minimum safe lidar clearance for actions


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _wall_lines(walls):
    """Return a list of Shapely LineStrings for all wall segments."""
    return [LineString([(w['x1'], w['y1']), (w['x2'], w['y2'])]) for w in walls]


def _outer_boundary(walls):
    """
    Build the outer boundary polygon from the maze walls.

    Prefers walls tagged type="boundary".  Falls back to all walls if none
    are tagged (older XML format).  Uses the convex hull of the selected
    wall endpoints — this is exact for convex environments (all current VPCE
    mazes are convex octagons) and requires no exact endpoint connectivity.
    """
    boundary_walls = [w for w in walls if w['wall_type'] == 'boundary']
    source         = boundary_walls if boundary_walls else walls

    endpoints = (
        [(w['x1'], w['y1']) for w in source] +
        [(w['x2'], w['y2']) for w in source]
    )
    return MultiPoint(endpoints).convex_hull


def _is_valid_point(x, y, boundary, all_lines, wall_clearance):
    """
    Return True if (x, y) is a valid robot start position:
      - inside the outer boundary polygon, and
      - at least wall_clearance metres from every wall segment (boundary and
        obstacle).
    """
    p = Point(x, y)
    if not boundary.contains(p):
        return False
    return all(p.distance(line) >= wall_clearance for line in all_lines)


# ---------------------------------------------------------------------------
# Plot helper
# ---------------------------------------------------------------------------

def _save_plot(df, walls, boundary, xml_path, spacing, wall_clearance):
    """Save a coverage plot to data/data_cache/<maze>_grid.png."""
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    maze_name = os.path.splitext(os.path.basename(xml_path))[0]
    out_path  = os.path.join('data', 'data_cache', f'{maze_name}_grid.png')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    fig, ax = plt.subplots(figsize=(7, 7))

    bx, by = boundary.exterior.xy
    ax.fill(bx, by, alpha=0.06, color='steelblue')
    ax.plot(bx, by, 'b--', lw=1, alpha=0.4)

    for w in walls:
        color = 'black' if w['wall_type'] == 'boundary' else 'saddlebrown'
        ax.plot([w['x1'], w['x2']], [w['y1'], w['y2']],
                color=color, lw=2.5, solid_capstyle='round')

    ax.scatter(df.x, df.y, s=6, color='steelblue', zorder=4)
    ax.set_title(
        f'{maze_name}  —  {len(df)} positions\n'
        f'spacing={spacing} m   clearance={wall_clearance} m'
    )
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(True, alpha=0.2)
    ax.legend(handles=[
        mpatches.Patch(color='black',       label='boundary wall'),
        mpatches.Patch(color='saddlebrown', label='obstacle wall'),
    ], fontsize=8)

    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"Plot saved  →  {out_path}")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def generate_grid(xml_path, spacing=0.1, wall_clearance=0.2,
                  theta=0.0, output=None, plot=False):
    """
    Generate a uniform grid of valid start positions for a REALM maze.

    Parameters
    ----------
    xml_path : str
        Path to the maze XML file.
    spacing : float
        Distance between adjacent grid points in metres.
        Smaller = denser coverage, more habituation positions.
        Default 0.1 m.
    wall_clearance : float
        Minimum distance from any wall surface to a grid point in metres.
        Default 0.2 m.
    theta : float
        Heading assigned to every generated position, in radians.
        Default 0.0.
    output : str or None
        If given, save the result to this CSV path (directories are created
        automatically).  Either way the DataFrame is returned.
    plot : bool
        If True, save a coverage plot to data/data_cache/<maze>_grid.png.
        Default False.

    Returns
    -------
    pd.DataFrame
        Columns: x, y, theta.
    """
    root  = ET.parse(xml_path).getroot()
    walls = parse_all_walls(root)

    all_lines = _wall_lines(walls)
    boundary  = _outer_boundary(walls)

    minx, miny, maxx, maxy = boundary.bounds
    xs = np.arange(minx, maxx + spacing, spacing)
    ys = np.arange(miny, maxy + spacing, spacing)

    points = [
        (round(float(x), 4), round(float(y), 4), theta)
        for x in xs
        for y in ys
        if _is_valid_point(x, y, boundary, all_lines, wall_clearance)
    ]

    if not points:
        raise ValueError(
            f"No valid positions found with wall_clearance={wall_clearance} m. "
            "Try reducing wall_clearance."
        )

    df = pd.DataFrame(points, columns=['x', 'y', 'theta'])

    if output:
        dir_name = os.path.dirname(output)
        if dir_name:
            os.makedirs(dir_name, exist_ok=True)
        df.to_csv(output, index=False)
        print(f"Saved {len(df)} positions  →  {output}")

    if plot:
        _save_plot(df, walls, boundary, xml_path, spacing, wall_clearance)

    return df


# ---------------------------------------------------------------------------
# Command-line interface
# ---------------------------------------------------------------------------

def _build_parser():
    p = argparse.ArgumentParser(
        description=(
            "Generate a CSV of valid robot start positions for a REALM maze XML.\n\n"
            "If --output is omitted the file is written alongside the XML as "
            "<maze>_positions.csv."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('xml',
                   help='Path to the maze XML file.')
    p.add_argument('--spacing', type=float, default=0.1,
                   help='Distance between grid points in metres (default: 0.1). '
                        'Smaller = denser coverage.')
    p.add_argument('--clearance', type=float, default=0.2,
                   help='Min distance from any wall in metres (default: 0.2).')
    p.add_argument('--theta', type=float, default=0.0,
                   help='Robot heading for all positions in radians (default: 0.0).')
    p.add_argument('--output', '-o', default=None,
                   help='Output CSV path.  Defaults to <maze>_train_points.csv '
                        'alongside the input XML as <maze>_positions.csv.')
    p.add_argument('--plot', action='store_true', default=False,
                   help='Save a coverage plot to data/data_cache/<maze>_grid.png.')
    return p


if __name__ == '__main__':
    args = _build_parser().parse_args()

    output = args.output
    if output is None:
        xml_dir   = os.path.dirname(args.xml)
        maze_name = os.path.splitext(os.path.basename(args.xml))[0]
        output    = os.path.join(xml_dir, 'positions', maze_name + '_positions.csv')

    generate_grid(
        xml_path=args.xml,
        spacing=args.spacing,
        wall_clearance=args.clearance,
        theta=args.theta,
        output=output,
        plot=args.plot,
    )
