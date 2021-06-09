# CMake generated Testfile for 
# Source directory: /home/ryan/course/src/Planner/jps3D
# Build directory: /home/ryan/course/src/Planner/jps3D/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_planner_2d "test_planner_2d" "/home/ryan/course/src/Planner/jps3D/data/corridor.yaml")
add_test(test_planner_3d "test_planner_3d" "/home/ryan/course/src/Planner/jps3D/data/simple3d.yaml")
add_test(test_distance_map_planner_2d "test_distance_map_planner_2d" "/home/ryan/course/src/Planner/jps3D/data/corridor.yaml")
