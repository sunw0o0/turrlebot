#!/usr/bin/env python3
"""
parking_node 가 카메라 영상(OpenCV)을 쓰도록 lane_follower 의 CMakeLists.txt 를 고친다.
(construction 빌드 설정이 이미 들어가 있어야 한다 = add_construction_build.py 를 먼저)
이미 고쳐져 있으면 아무것도 안 바꾼다.

  python3 ~/turrlebot/team_patch/add_parking_vision_build.py ~/Robit_intelligence_turtlebot_team
"""
import os
import re
import sys

root = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '.')
path = os.path.join(root, 'colcon_ws', 'src', 'lane_follower', 'CMakeLists.txt')
s = open(path).read()

if 'target_link_libraries(parking_node' in s:
    print('CMakeLists.txt: 이미 있음')
    sys.exit(0)

if not re.search(r'find_package\(\s*OpenCV\b', s) or not re.search(r'find_package\(\s*cv_bridge\b', s):
    print('CMakeLists.txt 에 OpenCV / cv_bridge 가 없음 -> add_construction_build.py 를 먼저 실행하세요')
    sys.exit(1)

# ament_target_dependencies(parking_node ...) 줄을 영상용으로 바꾸고, 그 앞에 OpenCV 링크 추가
new = ('target_link_libraries(parking_node ${OpenCV_LIBS})\n'
       'ament_target_dependencies(parking_node rclcpp geometry_msgs interfaces '
       'std_msgs sensor_msgs cv_bridge)')
s, n = re.subn(r'^ament_target_dependencies\(parking_node[^\n]*\)', new, s, count=1, flags=re.M)
if n == 0:
    print('CMakeLists.txt 에서 parking_node 줄을 못 찾음')
    sys.exit(1)
open(path, 'w').write(s)
print('CMakeLists.txt: parking_node 에 OpenCV / cv_bridge 추가함')
