#!/usr/bin/env python3

"""Generate the report/RViz occupancy map used alongside the topological planner.

The map deliberately does not replace the existing closed-loop controller.  Its
coordinate system is identical to Gazebo's ``world`` frame so that recorded
paths and calibrated orchard locations can be overlaid without reprojection.
"""

import argparse
from pathlib import Path


MIN_X, MAX_X = -11.0, 11.0
MIN_Y, MAX_Y = -17.0, 17.0
RESOLUTION = 0.05

# Calibrated crop-row centres.  A 1.10 m occupied strip leaves the five robot
# passages and both end-cap transfer corridors free.
CROP_ROW_X = (-6.561, -2.542, 1.479, 5.503)
CROP_MIN_Y, CROP_MAX_Y = -13.55, 13.35
CROP_HALF_WIDTH = 0.55
BOUNDARY_WIDTH = 0.20


def occupied(x, y):
    boundary = (
        x <= MIN_X + BOUNDARY_WIDTH
        or x >= MAX_X - BOUNDARY_WIDTH
        or y <= MIN_Y + BOUNDARY_WIDTH
        or y >= MAX_Y - BOUNDARY_WIDTH
    )
    crop = CROP_MIN_Y <= y <= CROP_MAX_Y and any(
        abs(x - centre) <= CROP_HALF_WIDTH for centre in CROP_ROW_X
    )
    return boundary or crop


def generate(path):
    width = round((MAX_X - MIN_X) / RESOLUTION)
    height = round((MAX_Y - MIN_Y) / RESOLUTION)
    pixels = bytearray(width * height)
    for image_row in range(height):
        # PGM starts at the upper-left; map_server's origin starts lower-left.
        y = MAX_Y - (image_row + 0.5) * RESOLUTION
        for column in range(width):
            x = MIN_X + (column + 0.5) * RESOLUTION
            pixels[image_row * width + column] = 0 if occupied(x, y) else 254
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write("P5\n%d %d\n255\n".encode() % (width, height))
        stream.write(pixels)
    print("Generated %s (%dx%d, %.2f m/cell)" % (path, width, height, RESOLUTION))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    generate(parser.parse_args().output)
