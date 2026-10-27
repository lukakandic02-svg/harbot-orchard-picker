cmake_minimum_required(VERSION 3.0.2)
project(harbot_orchard_picker)

find_package(catkin REQUIRED COMPONENTS rospy)

# ROS-free core (graph, fleet arbiter, geometry): src/harbot_orchard_picker
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/online_orchard_manager.py
  scripts/orchard_map_picker.py
  scripts/target_pick_executor.py
  scripts/orchard_map_picker_pick_eff.py
  scripts/arm_pick_demo.py
  scripts/fruit_pick_context.py
  scripts/report_metrics_recorder.py
  scripts/generate_dual_metrics.py
  scripts/generate_report_metrics.py
  scripts/orchard_map_visualizer.py
  scripts/passage_segment_check.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Plain library module imported by target_pick_executor.py (not a node).
install(FILES scripts/pick_eff_common.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps rviz config rddl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Unit tests (ROS-free) run with plain pytest (see README / CI).
