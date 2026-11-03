"""ROS 2 双节点联调脚本：同时拉起仿真发布端与航迹分析端，跑完任务剖面后收尾。

在 ros_humble 环境下运行（包的父目录，即 src/drone_trajectory_logger）::

    python tools/run_ros_demo.py
"""

import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.abspath(os.path.join(HERE, '..'))
MISSION_SECONDS = 23.0


def main():
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    env['RCUTILS_COLORIZED_OUTPUT'] = '0'

    pub = subprocess.Popen(
        [sys.executable, '-m', 'drone_trajectory_logger.main', '--once'],
        cwd=PKG_DIR, env=env,
        stdout=open(os.path.join(PKG_DIR, 'ros_pub_log.txt'), 'w', encoding='utf-8'),
        stderr=subprocess.STDOUT)
    ana = subprocess.Popen(
        [sys.executable, '-m', 'drone_trajectory_logger.trajectory_analyzer',
         '--no-plot', '--csv', 'ros_demo_trajectory.csv', '--exit-after', '24.5'],
        cwd=PKG_DIR, env=env,
        stdout=open(os.path.join(PKG_DIR, 'ros_analyzer_log.txt'), 'w', encoding='utf-8'),
        stderr=subprocess.STDOUT)

    print('两个节点已启动，运行 %.0f s 任务剖面 ...' % MISSION_SECONDS)
    deadline = time.monotonic() + MISSION_SECONDS + 15.0
    while time.monotonic() < deadline:
        if ana.poll() is not None:
            break
        time.sleep(1.0)

    for proc in (pub, ana):
        if proc.poll() is None:
            proc.terminate()
    try:
        pub.wait(timeout=10)
        ana.wait(timeout=10)
    except subprocess.TimeoutExpired:
        for proc in (pub, ana):
            proc.kill()

    print('--- 发布端日志（尾部）---')
    with open(os.path.join(PKG_DIR, 'ros_pub_log.txt'), encoding='utf-8') as f:
        print(''.join(f.readlines()[-12:]))
    print('--- 分析端日志（尾部）---')
    with open(os.path.join(PKG_DIR, 'ros_analyzer_log.txt'), encoding='utf-8') as f:
        print(''.join(f.readlines()[-30:]))


if __name__ == '__main__':
    main()
