"""Calibrated orchard geometry: simulator XY <-> symbolic (passage, index).

ROS-free so it can be unit-tested and reused by offline analysis tools.
"""

import json
import math


class OrchardGeometry:
    """Inverse calibration: simulator XY -> (passage, fractional index).

    Same geometry as OrchardMapPicker.load_tree_calibration(), from the same
    calibration file (reference trees T11, T18, T21), so the metrics recorder
    can log trajectories in map terms without the ROS node.

    Meant for reporting: off the passage lines (e.g. during a shortcut
    crossing) it returns the nearest passage/index.
    """

    def __init__(self, tree_points_file, num_rows, num_indices):
        points = {}
        with open(tree_points_file) as f:
            for line in f:
                if line.strip():
                    p = json.loads(line)
                    points[p["label"]] = p

        t11 = points["T11_row1_tree1"]
        t18 = points["T18_row1_tree8"]
        t21 = points["T21_row2_tree1"]

        self.t11 = (t11["x"], t11["y"])
        length_vec = (t18["x"] - self.t11[0], t18["y"] - self.t11[1])
        row_vec = (t21["x"] - self.t11[0], t21["y"] - self.t11[1])

        row_length = math.hypot(length_vec[0], length_vec[1])
        self.row_spacing = math.hypot(row_vec[0], row_vec[1])

        self.length_unit = (length_vec[0] / row_length, length_vec[1] / row_length)
        self.width_unit = (
            row_vec[0] / self.row_spacing, row_vec[1] / self.row_spacing
        )
        self.tree_spacing = row_length / float(num_indices - 1)
        self.num_rows = num_rows

    def xy_to_passage_index(self, x, y):
        rel_x, rel_y = x - self.t11[0], y - self.t11[1]
        frac_index = 1.0 + (
            rel_x * self.length_unit[0] + rel_y * self.length_unit[1]
        ) / self.tree_spacing
        passage_f = (
            rel_x * self.width_unit[0] + rel_y * self.width_unit[1]
        ) / self.row_spacing + 0.5
        passage = max(0, min(self.num_rows, round(passage_f)))
        return passage, frac_index
