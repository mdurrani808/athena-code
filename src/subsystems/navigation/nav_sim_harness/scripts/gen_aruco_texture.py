#!/usr/bin/env python3
"""One-shot dev tool: bake a DICT_4X4_50 ArUco tag PNG for the aruco_post model.

This is NOT part of the nav_sim_harness runtime and is never imported or called
at spawn time. Run it by hand when you need to (re)generate the tag texture that
the description/models/aruco_post model references, then commit the resulting PNG.

The marker uses the same dictionary the detection node expects:
    aruco_detection/aruco_node.py -> cv2.aruco.DICT_4X4_50

Layout of the generated image (so the printed/rendered face matches the model's
0.20 m face at 2.5 cm/cell):
    * 6x6-cell ArUco marker (4x4 data + 1-cell black border)  -> 15 cm
    * 1-cell white quiet zone padded on every side             -> +2.5 cm each
    * total 8x8 cells                                          -> 20 cm face

Usage:
    python3 gen_aruco_texture.py                # id=4 -> default model path
    python3 gen_aruco_texture.py --id 7
    python3 gen_aruco_texture.py --id 4 --out /tmp/tag.png --cell-px 120
"""
import argparse
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("OpenCV (cv2) is required. Run this inside the ROS2 devcontainer.")


# Default output: the aruco_post model's texture slot, resolved relative to this
# file. scripts/ -> nav_sim_harness -> navigation -> subsystems -> src (4 levels).
_THIS = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.normpath(os.path.join(_THIS, "..", "..", "..", ".."))
_DEFAULT_DIR = os.path.join(
    _SRC, "description", "models", "aruco_post", "materials", "textures"
)


def _get_dictionary():
    """Return the DICT_4X4_50 dictionary across new/old OpenCV aruco APIs."""
    aruco = cv2.aruco
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    # Older API (matches aruco_node.py's cv2.aruco.Dictionary_get usage)
    return aruco.Dictionary_get(aruco.DICT_4X4_50)


def _draw_marker(dictionary, marker_id, side_px):
    """Render the 6x6-cell marker (incl. black border) across new/old APIs."""
    aruco = cv2.aruco
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(dictionary, marker_id, side_px)
    return aruco.drawMarker(dictionary, marker_id, side_px)


def generate(marker_id, cell_px, out_path):
    dictionary = _get_dictionary()

    # The marker itself is 6 cells wide (4x4 data + 1-cell black border).
    marker_px = 6 * cell_px
    marker = _draw_marker(dictionary, marker_id, marker_px)

    # Pad a 1-cell white quiet zone on every side -> 8 cells total.
    pad = cell_px
    canvas = np.full(
        (marker_px + 2 * pad, marker_px + 2 * pad), 255, dtype=np.uint8
    )
    canvas[pad:pad + marker_px, pad:pad + marker_px] = marker

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if not cv2.imwrite(out_path, canvas):
        sys.exit(f"Failed to write {out_path}")
    print(f"Wrote {out_path}  ({canvas.shape[1]}x{canvas.shape[0]} px, id={marker_id})")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, default=4, dest="marker_id",
                    help="DICT_4X4_50 marker id to bake (default: 4)")
    ap.add_argument("--cell-px", type=int, default=100,
                    help="pixels per cell (default: 100 -> 800x800 px image)")
    ap.add_argument("--out", default=None,
                    help="output PNG path (default: aruco_post model texture slot)")
    args = ap.parse_args()

    if args.marker_id < 0 or args.marker_id >= 50:
        sys.exit("DICT_4X4_50 only defines ids 0..49")

    out_path = args.out or os.path.join(
        _DEFAULT_DIR, f"aruco_4x4_50_id{args.marker_id}.png"
    )
    generate(args.marker_id, args.cell_px, out_path)


if __name__ == "__main__":
    main()
