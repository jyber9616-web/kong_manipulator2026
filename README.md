# kong_manipulator2026
공주대 매니퓰레이터 수업

---  
0720  

--- 

wsl 설치   
github 동기화   
ros2 설치-jazzy (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)    
turtlesim turtle_node 실행해보기  
topic, action, service 공부  

---
2026-07-21 
---

1교시: 복습, ros2 common package  
2교시: 중요 컨셉(DDS, node spin, state)  
3교시: RMW architecture  
4교시: Node, Topic, Service, Action 개념  
5교시: 패키지 작성 ( ros2 pkg create)  
6교시: simple node 작성(publisher, subscriber)  
7교시: Header class time pub 작성, 5개 노드 실습  
8교시: 터틀심 움직이기  

---

2026-07-22 

---
1교시: 복습  
2교시: DDS wsl 에서 설정해야 할 내용 설명  
3교시: interface 정의, msg, srv 작성  
4교시: service thread server 작성  
5교시: service client 작성()  
6교시: parameter (add_on_set_parameter_callback)  
7교시: 외부 노드에서 parameter 변경 AsyncParameterClient  
8교시: launch 에서의 parameter 설정 Node(parameters=[])  

---

2026-07-23

---

1교시: 복습  
2교시: action interface 정의, action IDL fibonacci 작성  
3교시: topic, service, action 의 차이점  
4교시: action server 작성, action client 작성  
5교시: action thread server 작성( cancel, abort 구현)  
6교시: namespace 적용 launch 작성  
7교시: static tf 발행  
8교시: dynamic tf 발행  

---

2026-07-24

---

1교시: 복습  
2교시: tf2 설명 tf2 패키지 작성  
3교시: tf2 listener 작성, tf2 listener 에서 transform 받아오기  
4교시: 실습[터틀심 listener]  
5교시: turtlesim 에서 tf2 적용  
6교시: urdf 설명, urdf 패키지 작성  
7교시: xacro 실습  
8교시: urdf 실습  

---

2026-07-27 

---

1교시: 복습  
2교시: 하드웨어 연결 및 dynamixel wizard 설치 및 작동 테스트  
3교시: manipulator-X 패키지 설치 및 작동 실습  
4교시: descriptor 실행 및 tf-tree 확인  
5교시: bringup launch 실행 - robot_state_publisher, joint_state_publisher_gui, rviz2 실행  
6교시: teleo_keyboard 로 manipulator-X 제어 실습  
7교시: node 작성 trajectory_joint_state 로 manipulator-X 제어 실습  
8교시: 과제 - 춤추는 로봇 팔 만들기  

---

2026-07-28

---

1교시: 복습  
2교시: joint state action code 작성  
3교시: [실습] 춤추는 로봇팔 만들기  
4교시: teach manipulator 노드 작성 joint_states 구독  
5교시: teach manipulator 노드 작성 키보드 인식 코드, yaml 저장 파일  
6교시: play_recorded_dance 노드 작성  
7교시: pick and place 실습 (traching data 활용)  
8교시: moveit 실습  

---

2026-07-29 

---

1교시: 복습, moveit node class 작성  
2교시: moveit srdf 수정   
3교시: moveit RobotState로 매니퓰레이터 제어  
4교시: moveit rviz2 scence object 추가 및 planning scene 적용  
5교시: moveit scene monitor 로 scene object 추가 및 planning scene 적용  
6교시: wall 추가 해서 충돌 회피 경로 계획 실습  
7교시: moveit pick and place 실습(attachment object 활용)  
8교시: moveit pick and place 실습(자세 수정 및 디버깅)  

---

2026-07-30 

---

1교시: 복습  
2교시: moveit mini project - scene monitor 벽 사이 planning scene 적용  
3교시: moveit mini project - scene monitor 벽 사이 planning scene 적용  
4교시: thread class 작성 - moveitpy node 융합 class  
5교시: gazebo harmonic DOC 분석  
6교시: teleop_keyboard 로 gazebo harmonic 제어 실습  
7교시: gazebo 에 world 추가 및 수정  
8교시: gazebo 에 모델 등록 하고 사용하기 aruco_box 생성  

---

2026-07-31

---

1교시: 복습 gazebo spawn model  
2교시: 몸체 모델링 및 gazebo 에 적용  
3교시: gazebo plugin gz-sim-diff-drive 적용  
4교시: launch 파일 작성 (gz-sim, bridge)  
5교시: spawn model 로 gazebo 에 모델 등록 및 제어 실습  
6교시: vehicle model 에 카메라 추가 및 ros2 interface bridge 적용  
7교시: vehicle model tf 발행 -> rviz2 확인  
8교시: manipulator-X 에 카메라 추가 및 ros2 interface bridge 적용  

---

2026-08-03

---

1교시: 복습 opencv 설치  
2교시: 영상처리의 이해, ros2 node + opencv  
3교시: imread, imwrite, jpg 압축 저장 image pub sug 구현  
4교시: compressed img pub 구현  
5교시: BGR2RGB 구현  
6교시: camera 영상 처리, mjpg, gstreamer pipeline 구현  
7교시: draw rectangle, draw line, background  
8교시: draw circle  

---

2026-08-04 

---

1교시: 복습, ellipse  
2교시: event mouse, keyboard 예제  
3교시: ros2 event 통합 예제  
4교시: mask 연산, bitwise_and, bitwise_or, bitwise_not, copyTo  
5교시: reshape, flatten, pixel 연산  
6교시: roi deep-copy  
7교시: color space conversion, inrange trackbar 실습  
8교시: manipulator-X gazebo 빨간 공 찾기 실습- model 작성, 노드 작성  
