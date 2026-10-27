#!/usr/bin/env python3

"""Publish calibrated orchard graph nodes, edges and dual-work-zone colours."""

import json
from pathlib import Path

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def marker(marker_id, marker_type, namespace):
    result = Marker()
    result.header.frame_id = "map"
    result.header.stamp = rospy.Time.now()
    result.ns = namespace
    result.id = marker_id
    result.type = marker_type
    result.action = Marker.ADD
    result.pose.orientation.w = 1.0
    result.lifetime = rospy.Duration(0)
    return result


def zone_for(passage, index):
    if passage <= 1 or (passage == 2 and index <= 4):
        return "robot"
    return "robot_b"


def main():
    rospy.init_node("orchard_map_visualizer")
    map_path = Path(rospy.get_param(
        "~orchard_map",
        str(Path.home() / "harbot_calibration/orchard_map.json"),
    ))
    if not map_path.exists():
        rospy.logfatal("Orchard calibration map does not exist: %s", map_path)
        raise SystemExit(2)
    data = json.loads(map_path.read_text())
    publisher = rospy.Publisher("/orchard_map_markers", MarkerArray, queue_size=1, latch=True)
    output = MarkerArray()

    locations = {}
    for passage_name, entries in sorted(data["passages"].items()):
        passage = int(passage_name[1:])
        for index_text, entry in sorted(entries.items(), key=lambda item: int(item[0])):
            index = int(index_text)
            locations[(passage, index)] = (entry["x"], entry["y"])

            node = marker(100 * passage + index, Marker.SPHERE, "zone_nodes")
            node.pose.position.x, node.pose.position.y = entry["x"], entry["y"]
            node.pose.position.z = 0.10
            node.scale.x = node.scale.y = node.scale.z = 0.22
            node.color.a = 0.95
            if zone_for(passage, index) == "robot":
                node.color.r, node.color.g, node.color.b = 0.10, 0.45, 1.00
            else:
                node.color.r, node.color.g, node.color.b = 1.00, 0.35, 0.10
            output.markers.append(node)

            label = marker(1000 + 100 * passage + index, Marker.TEXT_VIEW_FACING, "location_labels")
            label.pose.position.x, label.pose.position.y = entry["x"], entry["y"]
            label.pose.position.z = 0.45
            label.scale.z = 0.24
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = "P%d_I%d" % (passage, index)
            output.markers.append(label)

    edges = marker(5000, Marker.LINE_LIST, "topological_edges")
    edges.scale.x = 0.045
    edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.15, 0.85, 0.25, 0.8
    for passage in range(5):
        for index in range(1, 8):
            for key in ((passage, index), (passage, index + 1)):
                x, y = locations[key]
                edges.points.append(Point(x=x, y=y, z=0.04))
    for passage in range(4):
        for index in (1, 8):
            for key in ((passage, index), (passage + 1, index)):
                x, y = locations[key]
                edges.points.append(Point(x=x, y=y, z=0.04))
    output.markers.append(edges)

    publisher.publish(output)
    rospy.loginfo(
        "Published orchard map overlay: %d locations, blue=robot, orange=robot_b, source=%s",
        len(locations), map_path,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
