# 녹화(ros2 bag) 안의 로그 (/rosout) 를 시간 순으로 꺼낸다. 시간은 녹화 시작부터 [초] (send.sh 의 사진 이름과 같은 기준)
#   사용: cd ~/rec && python3 ~/turrlebot/team_patch/bag_log.py park10 > park10_frames/log.txt
#   (먼저 source ~/Robit_intelligence_turtlebot_team/colcon_ws/install/setup.bash)
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log

bag = sys.argv[1]
info = rosbag2_py.Info().read_metadata(bag, '')
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=info.storage_identifier),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
t0 = None
lines = 0
while reader.has_next():
    topic, data, t = reader.read_next()
    if t0 is None:
        t0 = t                      # 녹화의 첫 메시지 = 시작
    if topic != '/rosout':
        continue
    m = deserialize_message(data, Log)
    print(f'{(t - t0) / 1e9:8.3f} [{m.name}] {m.msg}')
    lines += 1
print(f'로그 {lines}줄', file=sys.stderr)
