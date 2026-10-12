#!/usr/bin/env python3
"""
lane_follower 의 CMakeLists.txt / package.xml 에 construction_node 빌드 설정을 넣는다.
construction.patch 가 이 두 파일에서 실패할 때 쓴다 (이미 들어가 있으면 아무것도 안 바꾼다).

  python3 ~/turrlebot/team_patch/add_construction_build.py ~/Robit_intelligence_turtlebot_team
"""
import os
import re
import sys

root = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '.')
pkg = os.path.join(root, 'colcon_ws', 'src', 'lane_follower')

# ---- CMakeLists.txt ----
path = os.path.join(pkg, 'CMakeLists.txt')
s = open(path).read()
if 'construction_node' in s:
    print('CMakeLists.txt: 이미 있음')
else:
    # 1) find_package 추가 (마지막 find_package 줄 뒤에, 없는 것만)
    need = ['std_msgs', 'sensor_msgs', 'cv_bridge']
    lines = []
    for p in need:
        if not re.search(r'find_package\(\s*%s\b' % p, s):
            lines.append('find_package(%s REQUIRED)' % p)
    if not re.search(r'find_package\(\s*OpenCV\b', s):
        lines.append('find_package(OpenCV REQUIRED COMPONENTS core imgproc)')
    last = list(re.finditer(r'^find_package\(.*\)\s*$', s, re.M))[-1]
    s = s[:last.end()] + '\n' + '\n'.join(lines) + s[last.end():]

    # 2) 실행 파일 추가 (install(TARGETS 앞에)
    block = ('# 공사 구간 노드: 공사 표지판을 보면 영상으로 장애물과 차선을 보고 피해 간다\n'
             'add_executable(construction_node src/construction_node.cpp)\n'
             'target_link_libraries(construction_node ${OpenCV_LIBS})\n'
             'ament_target_dependencies(construction_node rclcpp geometry_msgs interfaces '
             'std_msgs sensor_msgs cv_bridge)\n\n')
    m = re.search(r'^install\(TARGETS', s, re.M)
    s = s[:m.start()] + block + s[m.start():]

    # 3) install(TARGETS ... ) 에 construction_node 넣기
    s = re.sub(r'^install\(TARGETS([^\n]*)', lambda mm: 'install(TARGETS' + mm.group(1).rstrip() +
               ' construction_node', s, count=1, flags=re.M)
    open(path, 'w').write(s)
    print('CMakeLists.txt: construction_node 추가함')

# ---- package.xml ----
path = os.path.join(pkg, 'package.xml')
s = open(path).read()
added = []
for dep in ['std_msgs', 'sensor_msgs', 'cv_bridge', 'libopencv-dev']:
    if '<depend>%s</depend>' % dep not in s:
        added.append('  <depend>%s</depend>' % dep)
if added:
    m = list(re.finditer(r'^\s*<depend>[^<]+</depend>\s*$', s, re.M))[-1]
    s = s[:m.end()] + '\n' + '\n'.join(added) + s[m.end():]
    open(path, 'w').write(s)
    print('package.xml: 추가함 ->', ', '.join(a.strip() for a in added))
else:
    print('package.xml: 이미 있음')
