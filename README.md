# RoboMaster TT

![Static Badge](https://img.shields.io/badge/任务流程-blue)

![Static Badge](https://img.shields.io/badge/起飞-red)

```bash
roslaunch rmtt_driver rmtt_bringup.launch
roslaunch rmtt_teleop rmtt_teleop_key.launch
```

![Static Badge](https://img.shields.io/badge/有限状态机-green)

```bash
rosrun rmtt_tracker rmtt_smach.py
```

![Static Badge](https://img.shields.io/badge/识别跟踪apriltag-orange)

```bash
rosrun rmtt_tracker rmtt_tag_tracker.py
```

![Static Badge](https://img.shields.io/badge/定位毯识别-black)

```bash
roslaunch rmtt_apriltag detection.launch
```

![Static Badge](https://img.shields.io/badge/穿越环-purple)

```bash
rosrun rmtt_tracker rmtt_circle_tracker.py
rosrun rmtt_tracker rmtt_circle_detection.py
```

![Static Badge](https://img.shields.io/badge/路径规划-gray)

```bash
rosrun rmtt_tracker stanley.py
```

[技术报告](./智能无人系统综合设计.pdf)