#!/usr/bin/env python3
"""
WiFi Signal Heatmap Visualizer
================================
Reads the CSV from wifi_signal_logger_ros2.py and generates a smooth
heatmap of signal strength overlaid on the Nav2 map.

Color scale:
  BLUE   = weak signal  (e.g. -90 dBm)
  RED    = strong signal (e.g. -30 dBm)

Usage:
  # Basic heatmap (no map background)
  python3 wifi_heatmap.py wifi_signal_log.csv

  # Overlay on your Nav2 map
  python3 wifi_heatmap.py wifi_signal_log.csv --map /path/to/map.pgm --yaml /path/to/map.yaml

  # Live mode — re-renders every N seconds while logger is running
  python3 wifi_heatmap.py wifi_signal_log.csv --live --refresh 5

  # Save to file instead of displaying
  python3 wifi_heatmap.py wifi_signal_log.csv --save heatmap.png --dpi 300
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.colors import LinearSegmentedColormap
from scipy.interpolate import griddata
import yaml as pyyaml  # pip install pyyaml


# ---------------------------------------------------------------------------
# Custom colormap: blue (weak) → cyan → green → yellow → red (strong)
# ---------------------------------------------------------------------------
WIFI_CMAP = LinearSegmentedColormap.from_list(
    "wifi_signal",
    [
        (0.0,  "#0000FF"),  # -90 dBm  deep blue (very weak / dead zone)
        (0.15, "#0066FF"),  # blue
        (0.30, "#00CCFF"),  # cyan
        (0.45, "#00FF88"),  # green-cyan
        (0.60, "#66FF00"),  # green-yellow
        (0.75, "#FFCC00"),  # yellow-orange
        (0.90, "#FF4400"),  # orange-red
        (1.0,  "#FF0000"),  # -30 dBm  red (excellent signal)
    ],
)


def load_nav2_map(pgm_path: str, yaml_path: str):
    """
    Load the Nav2/SLAM map (.pgm + .yaml) and return the image
    array along with origin and resolution for coordinate mapping.
    """
    with open(yaml_path, "r") as f:
        meta = pyyaml.safe_load(f)

    resolution = meta["resolution"]          # meters per pixel
    origin = meta["origin"]                  # [x, y, theta]
    occupied_thresh = meta.get("occupied_thresh", 0.65)
    free_thresh = meta.get("free_thresh", 0.196)

    map_img = plt.imread(pgm_path)

    # If the image is already uint8 0-255, normalize to 0-1
    if map_img.dtype == np.uint8:
        map_img = map_img.astype(float) / 255.0

    # PGM has row 0 at top, but Nav2 origin is bottom-left.
    # Flip so it aligns with origin="lower" in matplotlib.
    map_img = np.flipud(map_img)

    return map_img, resolution, origin


def world_to_pixel(x, y, origin, resolution):
    """Convert Nav2 map-frame coordinates to pixel coordinates."""
    px = (x - origin[0]) / resolution
    py = (y - origin[1]) / resolution
    return px, py


def build_heatmap(
    df: pd.DataFrame,
    map_img=None,
    resolution=None,
    origin=None,
    grid_res: float = 0.1,
    vmin: float = -90,
    vmax: float = -30,
    interp_method: str = "cubic",
    radius: float = None,
):
    """
    Build and return a matplotlib figure with the WiFi heatmap.

    Parameters
    ----------
    df : DataFrame with columns x, y, signal_dbm
    map_img : optional Nav2 map image array
    resolution : map resolution (m/px)
    origin : map origin [x, y, theta]
    grid_res : interpolation grid resolution in meters
    vmin/vmax : signal range for colormap
    interp_method : 'cubic', 'linear', or 'nearest'
    radius : max interpolation radius (None = auto)
    """
    x = df["x"].values
    y = df["y"].values
    signal = df["signal_dbm"].values

    # Build interpolation grid
    margin = 1.0  # 1m padding
    xi = np.arange(x.min() - margin, x.max() + margin, grid_res)
    yi = np.arange(y.min() - margin, y.max() + margin, grid_res)
    xi_grid, yi_grid = np.meshgrid(xi, yi)

    # Interpolate signal strength onto grid
    zi_grid = griddata(
        points=(x, y),
        values=signal,
        xi=(xi_grid, yi_grid),
        method=interp_method,
    )

    # If cubic leaves NaNs at edges, fill with nearest-neighbor
    if np.any(np.isnan(zi_grid)):
        zi_nearest = griddata(
            points=(x, y),
            values=signal,
            xi=(xi_grid, yi_grid),
            method="nearest",
        )
        mask = np.isnan(zi_grid)
        zi_grid[mask] = zi_nearest[mask]

    # Optionally mask areas far from any data point
    if radius is not None:
        from scipy.spatial import cKDTree
        tree = cKDTree(np.column_stack([x, y]))
        grid_points = np.column_stack([xi_grid.ravel(), yi_grid.ravel()])
        dists, _ = tree.query(grid_points)
        far_mask = dists.reshape(xi_grid.shape) > radius
        zi_grid[far_mask] = np.nan

    # Clamp to range
    zi_grid = np.clip(zi_grid, vmin, vmax)

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(14, 10))

    # Draw Nav2 map as background if provided
    if map_img is not None and resolution is not None and origin is not None:
        h, w = map_img.shape[:2]
        extent_map = [
            origin[0],
            origin[0] + w * resolution,
            origin[1],
            origin[1] + h * resolution,
        ]
        # Show map in grayscale underneath
        ax.imshow(
            map_img,
            cmap="gray",
            origin="lower",
            extent=extent_map,
            alpha=0.6,
            zorder=0,
        )

    # Draw heatmap
    norm = mcolors.Normalize(vmin=vmin, vmax=vmax)
    extent_heat = [xi.min(), xi.max(), yi.min(), yi.max()]

    im = ax.imshow(
        zi_grid,
        cmap=WIFI_CMAP,
        norm=norm,
        origin="lower",
        extent=extent_heat,
        alpha=0.7,
        interpolation="bilinear",
        zorder=1,
    )

    # Draw rover path as dots
    scatter = ax.scatter(
        x, y,
        c=signal,
        cmap=WIFI_CMAP,
        norm=norm,
        s=12,
        edgecolors="black",
        linewidths=0.3,
        zorder=2,
        label="Sample points",
    )

    # Draw start / end markers
    ax.plot(x[0], y[0], "^", color="white", markersize=12,
            markeredgecolor="black", markeredgewidth=1.5, zorder=3, label="Start")
    ax.plot(x[-1], y[-1], "s", color="white", markersize=10,
            markeredgecolor="black", markeredgewidth=1.5, zorder=3, label="End")

    # Colorbar
    cbar = fig.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label("Signal Strength (dBm)", fontsize=12)
    cbar.ax.set_yticks(np.arange(vmin, vmax + 1, 10))

    # Add signal quality labels on colorbar
    quality_labels = {
        -30: "Excellent",
        -50: "Good",
        -60: "Fair",
        -70: "Weak",
        -80: "Very Weak",
        -90: "Dead Zone",
    }
    for dbm_val, label in quality_labels.items():
        if vmin <= dbm_val <= vmax:
            cbar.ax.annotate(
                f"  {label}",
                xy=(1.0, dbm_val),
                xycoords=("axes fraction", "data"),
                fontsize=8,
                va="center",
                color="#444444",
            )

    ax.set_xlabel("X (meters)", fontsize=12)
    ax.set_ylabel("Y (meters)", fontsize=12)
    ax.set_title("WiFi Signal Strength Heatmap", fontsize=16, fontweight="bold")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2)

    # Stats box
    stats_text = (
        f"Samples: {len(df)}\n"
        f"Mean: {signal.mean():.1f} dBm\n"
        f"Min:  {signal.min():.0f} dBm\n"
        f"Max:  {signal.max():.0f} dBm\n"
        f"Area: {(x.max()-x.min()):.1f} × {(y.max()-y.min()):.1f} m"
    )
    ax.text(
        0.98, 0.02, stats_text,
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="bottom",
        horizontalalignment="right",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85),
        fontfamily="monospace",
        zorder=5,
    )

    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="WiFi signal heatmap visualizer")
    parser.add_argument("csv", help="Path to wifi_signal_log.csv")
    parser.add_argument("--map", help="Path to Nav2 map .pgm file")
    parser.add_argument("--yaml", help="Path to Nav2 map .yaml file")
    parser.add_argument("--grid-res", type=float, default=0.1,
                        help="Interpolation grid resolution in meters (default: 0.1)")
    parser.add_argument("--vmin", type=float, default=-90,
                        help="Min signal for color scale (default: -90 dBm)")
    parser.add_argument("--vmax", type=float, default=-30,
                        help="Max signal for color scale (default: -30 dBm)")
    parser.add_argument("--radius", type=float, default=None,
                        help="Max interpolation radius in meters (masks far areas)")
    parser.add_argument("--save", help="Save to file instead of displaying")
    parser.add_argument("--dpi", type=int, default=200, help="DPI for saved image")
    parser.add_argument("--live", action="store_true",
                        help="Live mode — re-render periodically")
    parser.add_argument("--refresh", type=float, default=5.0,
                        help="Refresh interval in seconds for live mode")

    args = parser.parse_args()

    # Load Nav2 map if provided
    map_img, resolution, origin = None, None, None
    if args.map and args.yaml:
        print(f"Loading Nav2 map: {args.map}")
        map_img, resolution, origin = load_nav2_map(args.map, args.yaml)
        print(f"  Resolution: {resolution} m/px, Origin: {origin}")

    def render_once():
        df = pd.read_csv(args.csv)

        if "signal_dbm" not in df.columns or df["signal_dbm"].dropna().empty:
            print("No signal data found yet. Waiting...")
            return None

        df = df.dropna(subset=["x", "y", "signal_dbm"])
        if len(df) < 4:
            print(f"Only {len(df)} samples — need at least 4 for interpolation.")
            return None

        print(f"Rendering heatmap with {len(df)} samples...")
        fig = build_heatmap(
            df,
            map_img=map_img,
            resolution=resolution,
            origin=origin,
            grid_res=args.grid_res,
            vmin=args.vmin,
            vmax=args.vmax,
            radius=args.radius,
        )
        return fig

    if args.live:
        # Live mode: keep re-rendering
        plt.ion()
        print(f"Live mode — refreshing every {args.refresh}s (Ctrl+C to stop)")
        while True:
            try:
                fig = render_once()
                if fig is not None:
                    if args.save:
                        fig.savefig(args.save, dpi=args.dpi, bbox_inches="tight")
                        print(f"  Saved → {args.save}")
                    else:
                        plt.show()
                        plt.pause(args.refresh)
                    plt.close(fig)
                else:
                    time.sleep(args.refresh)
            except KeyboardInterrupt:
                print("\nStopped.")
                break
    else:
        # One-shot render
        fig = render_once()
        if fig is None:
            sys.exit(1)
        if args.save:
            fig.savefig(args.save, dpi=args.dpi, bbox_inches="tight")
            print(f"Saved → {args.save}")
        else:
            plt.show()


if __name__ == "__main__":
    main()
