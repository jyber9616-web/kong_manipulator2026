# Add Camera

---

- /home/ju/kong_manipulator2026/open_manipulator_ws/src/open_manipulator/open_manipulator_bringup/launch/open_manipulator_x_gazebo.launch.py 파일 수정
- 149 라인부터
```python
bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/gripper_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/gripper_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        output="screen",
    )
```

- /home/ju/kong_manipulator2026/open_manipulator_ws/src/open_manipulator/open_manipulator_bringup/worlds/empty_world.sdf 파일 수정
- 13번 라인부터

```xml
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
```

- /home/ju/kong_manipulator2026/open_manipulator_ws/src/open_manipulator/open_manipulator_description/gazebo/open_manipulator_x.gazebo.xacro 파일 수정
- 43번 라인부터

```xml
<gazebo reference="${prefix}camera_link">
    <sensor name="gripper_camera" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>/gripper_camera/image_raw</topic>
        <camera>
            <horizontal_fov>1.0472</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.05</near>
                <far>100.0</far>
            </clip>
        </camera>
    </sensor>
  </gazebo>
```

- /home/ju/kong_manipulator2026/open_manipulator_ws/src/open_manipulator/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro 파일수정
- 182번 라인부터

```xml
<joint name="${prefix}camera_joint" type="fixed">
      <parent link="${prefix}link5"/>
      <child link="${prefix}camera_link"/>
      <origin xyz="0.07 0  0.05" rpy="0 0 0"/>
    </joint>

    <link name="${prefix}camera_link">
        <inertial>
            <mass value="0.05"/>
            <inertia ixx="1.0e-5" ixy="0" ixz="0" iyy="1.0e-5" iyz="0" izz="1.0e-5"/>
        </inertial>
        <visual>
            <geometry>
                <box size="0.04 0.03 0.025"/>
            </geometry>
            <material name="camera_black">
              <color rgba="0.05 0.05 0.05 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.04 0.03 0.025"/>
            </geometry>
        </collision>
    </link>
```

