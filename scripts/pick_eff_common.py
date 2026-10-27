#!/usr/bin/env python3

import csv
from pathlib import Path


TARGET_META = Path.home() / "harbot_rddl/orchard_targets_meta.csv"


def parse_loc(loc):
    left, right = loc.split("_")
    passage = int(left.replace("P", ""))
    index = int(right.replace("I", ""))
    return passage, index


def load_targets():
    if not TARGET_META.exists():
        raise RuntimeError("Missing target meta CSV: %s" % TARGET_META)

    targets = {}
    with open(TARGET_META) as f:
        reader = csv.DictReader(f)
        for row in reader:
            target = row["target"].strip()
            loc = row["loc"].strip()
            passage, index = parse_loc(loc)

            targets[target] = {
                "target": target,
                "loc": loc,
                "passage": passage,
                "index": index,
                "row": int(row["row"]),
                "side": row["side"].strip(),
            }

    return dict(sorted(targets.items(), key=lambda kv: int(kv[0][1:])))
