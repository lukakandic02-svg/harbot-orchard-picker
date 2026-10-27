#!/usr/bin/env python3

"""Navigation-only test for a continuous segment in any orchard passage."""

import sys
from pathlib import Path

import rospy

sys.path.insert(0, str(Path(__file__).resolve().parent))

from orchard_map_picker import OrchardMapPicker


def main():
    rospy.init_node("test_continuous_passage_segment")

    passage = int(rospy.get_param("~passage", 0))
    start_index = int(rospy.get_param("~start_index", 1))
    end_index = int(rospy.get_param("~end_index", 5))
    position_from_passage = int(rospy.get_param(
        "~position_from_passage", passage
    ))

    picker = OrchardMapPicker()
    picker.wait_for_odom()

    if passage < 0 or passage > picker.num_rows:
        rospy.logerr(
            "TEST FAILED invalid passage=P%d valid_range=P0..P%d",
            passage,
            picker.num_rows
        )
        return 1

    if not (1 <= start_index <= picker.num_indices):
        rospy.logerr(
            "TEST FAILED invalid start_index=I%d valid_range=I1..I%d",
            start_index,
            picker.num_indices
        )
        return 1

    if not (1 <= end_index <= picker.num_indices):
        rospy.logerr(
            "TEST FAILED invalid end_index=I%d valid_range=I1..I%d",
            end_index,
            picker.num_indices
        )
        return 1

    if start_index == end_index:
        rospy.logerr("TEST FAILED start and end index are both I%d", start_index)
        return 1
    if position_from_passage < 0 or position_from_passage > picker.num_rows:
        rospy.logerr(
            "TEST FAILED invalid position_from_passage=P%d",
            position_from_passage
        )
        return 1
    if position_from_passage != passage and start_index not in (1, picker.num_indices):
        rospy.logerr(
            "TEST FAILED passage transfer requires endcap start I1 or I%d",
            picker.num_indices
        )
        return 1

    rospy.loginfo("=" * 60)
    rospy.loginfo(
        "CONTINUOUS PASSAGE TEST passage=P%d start=I%d end=I%d",
        passage,
        start_index,
        end_index
    )
    rospy.loginfo("=" * 60)

    if position_from_passage != passage:
        rospy.loginfo(
            "TEST safe positioning from=P%d to=P%d through endcap=I%d",
            position_from_passage,
            passage,
            start_index
        )
        if not picker.go_to_station(position_from_passage, start_index):
            picker.stop()
            rospy.logerr("TEST FAILED could not reach source endcap station")
            return 1

        passage_step = 1 if passage > position_from_passage else -1
        transfer_direction = 1 if start_index == picker.num_indices else -1
        current_passage = position_from_passage
        while current_passage != passage:
            next_passage = current_passage + passage_step
            if not picker.transfer_to_next_passage(
                current_passage,
                next_passage,
                transfer_direction
            ):
                picker.stop()
                rospy.logerr(
                    "TEST FAILED safe transfer P%d -> P%d",
                    current_passage,
                    next_passage
                )
                return 1
            current_passage = next_passage

    rospy.loginfo("TEST positioning at start station P%d_I%d", passage, start_index)
    if not picker.go_to_station(passage, start_index):
        picker.stop()
        rospy.logerr("TEST FAILED could not reach start station")
        return 1

    start_x = picker.x
    start_y = picker.y
    rospy.loginfo(
        "TEST start reached current=(%.3f, %.3f, %.3f)",
        picker.x,
        picker.y,
        picker.yaw
    )

    if not picker.follow_passage_segment(passage, start_index, end_index):
        picker.stop()
        rospy.logerr("TEST FAILED continuous passage motion")
        return 1

    target_x, target_y = picker.passage_pose(passage, end_index)
    final_error = ((target_x - picker.x) ** 2 + (target_y - picker.y) ** 2) ** 0.5
    traveled = ((picker.x - start_x) ** 2 + (picker.y - start_y) ** 2) ** 0.5

    picker.stop()
    rospy.loginfo(
        "TEST SUCCESS passage=P%d start=I%d end=I%d traveled=%.3f final_error=%.3f "
        "current=(%.3f, %.3f, %.3f)",
        passage,
        start_index,
        end_index,
        traveled,
        final_error,
        picker.x,
        picker.y,
        picker.yaw
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
