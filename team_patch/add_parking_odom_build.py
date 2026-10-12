#!/usr/bin/env python3
"""
main 브랜치(+ 예전 주차/공사 패치)에서 새 parking_node(odom 사용)를 빌드할 수 있게
lane_follower 의 CMakeLists.txt / package.xml 에 nav_msgs, OpenCV, cv_bridge 를 넣는다.
이미 들어가 있으면 아무것도 안 바꾼다.

  python3 ~/turrlebot/team_patch/add_parking_odom_build.py ~/Robit_intelligence_turtlebot_team
"""
import os
import re
import sys

root = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '.')
pkg = os.path.join(root, 'colcon_ws', 'src', 'lane_follower')

# ---- CMakeLists.txt ----
path = os.path.join(pkg, 'CMakeLists.txt')
s = open(path).read()

# 1) find_package: 없는 것만 마지막 find_package 줄 뒤에 추가
need = []
for name, line in [('nav_msgs', 'find_package(nav_msgs REQUIRED)'),
                   ('sensor_msgs', 'find_package(sensor_msgs REQUIRED)'),
                   ('cv_bridge', 'find_package(cv_bridge REQUIRED)'),
                   ('OpenCV', 'find_package(OpenCV REQUIRED COMPONENTS core imgproc)')]:
    if not re.search(r'find_package\(\s*%s\b' % name, s):
        need.append(line)
if need:
    last = list(re.finditer(r'^find_package\(.*\)\s*$', s, re.M))[-1]
    s = s[:last.end()] + '\n' + '\n'.join(need) + s[last.end():]

# 2) parking_node 실행 파일 설정
block = ('target_link_libraries(parking_node ${OpenCV_LIBS})\n'
         'ament_target_dependencies(parking_node rclcpp geometry_msgs sensor_msgs nav_msgs '
         'interfaces cv_bridge)')
if 'add_executable(parking_node' not in s:
    m = re.search(r'^install\(TARGETS', s, re.M)
    s = s[:m.start()] + 'add_executable(parking_node src/parking_node.cpp)\n' + block + '\n\n' + s[m.start():]
    s = re.sub(r'^install\(TARGETS([^\n]*)', lambda mm: 'install(TARGETS' + mm.group(1).rstrip() +
               ' parking_node', s, count=1, flags=re.M)
else:
    s = re.sub(r'^target_link_libraries\(parking_node[^\n]*\)\n', '', s, flags=re.M)
    s = re.sub(r'^ament_target_dependencies\(parking_node[^\n]*\)', block, s, count=1, flags=re.M)
open(path, 'w').write(s)
print('CMakeLists.txt: parking_node 에 nav_msgs / OpenCV / cv_bridge 연결')

# ---- package.xml ----
path = os.path.join(pkg, 'package.xml')
s = open(path).read()
added = []
for dep in ['nav_msgs', 'sensor_msgs', 'cv_bridge', 'libopencv-dev']:
    if '<depend>%s</depend>' % dep not in s:
        added.append('  <depend>%s</depend>' % dep)
if added:
    m = list(re.finditer(r'^\s*<depend>[^<]+</depend>\s*$', s, re.M))[-1]
    s = s[:m.end()] + '\n' + '\n'.join(added) + s[m.end():]
    open(path, 'w').write(s)
    print('package.xml: 추가함 ->', ', '.join(a.strip() for a in added))
else:
    print('package.xml: 이미 있음')
