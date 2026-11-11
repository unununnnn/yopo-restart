"""生成一次飞行的 roslaunch。不改 ego-planner-swarm 里的文件。

不启动 rviz，也不启动 random_forest。圆柱点云由 ros_record.py 发到
``/map_generator/global_cloud``。odom_visualization 不启动。
"""

from __future__ import annotations

from .assumptions import (
    MAX_ACC_MPS2,
    MAX_VEL_MPS,
    PLANNER_MAP_X_M,
    PLANNER_MAP_Y_M,
    PLANNER_MAP_Z_M,
    PLANNING_HORIZON_M,
)


def launch_xml(scenario: dict) -> str:
    drones = []
    for d in scenario["drones"]:
        drones.append(_one_drone(int(d["ros_id"]), d["start_enu"], d["goal_enu"]))
    body = "\n".join(drones)
    return f"""<launch>
  <!-- 地图边长与障碍半径是假设，见 scripts/teacher/assumptions.py。 -->
  {body}
</launch>
"""


def _one_drone(drone_id: int, start, goal) -> str:
    sx, sy, sz = (float(v) for v in start)
    gx, gy, gz = (float(v) for v in goal)
    return f"""
  <include file="$(find ego_planner)/launch/advanced_param.xml">
    <arg name="drone_id" value="{drone_id}"/>
    <arg name="map_size_x_" value="{PLANNER_MAP_X_M}"/>
    <arg name="map_size_y_" value="{PLANNER_MAP_Y_M}"/>
    <arg name="map_size_z_" value="{PLANNER_MAP_Z_M}"/>
    <arg name="odometry_topic" value="visual_slam/odom"/>
    <arg name="camera_pose_topic" value="pcl_render_node/camera_pose"/>
    <arg name="depth_topic" value="pcl_render_node/depth"/>
    <arg name="cloud_topic" value="pcl_render_node/cloud"/>
    <arg name="cx" value="321.04638671875"/>
    <arg name="cy" value="243.44969177246094"/>
    <arg name="fx" value="387.229248046875"/>
    <arg name="fy" value="387.229248046875"/>
    <arg name="max_vel" value="{MAX_VEL_MPS}"/>
    <arg name="max_acc" value="{MAX_ACC_MPS2}"/>
    <arg name="planning_horizon" value="{PLANNING_HORIZON_M}"/>
    <arg name="flight_type" value="2"/>
    <arg name="use_distinctive_trajs" value="true"/>
    <arg name="obj_num_set" value="10"/>
    <arg name="point_num" value="1"/>
    <arg name="point0_x" value="{gx}"/>
    <arg name="point0_y" value="{gy}"/>
    <arg name="point0_z" value="{gz}"/>
    <arg name="point1_x" value="0.0"/>
    <arg name="point1_y" value="0.0"/>
    <arg name="point1_z" value="1.0"/>
    <arg name="point2_x" value="0.0"/>
    <arg name="point2_y" value="0.0"/>
    <arg name="point2_z" value="1.0"/>
    <arg name="point3_x" value="0.0"/>
    <arg name="point3_y" value="0.0"/>
    <arg name="point3_z" value="1.0"/>
    <arg name="point4_x" value="0.0"/>
    <arg name="point4_y" value="0.0"/>
    <arg name="point4_z" value="1.0"/>
  </include>

  <node pkg="ego_planner" name="drone_{drone_id}_traj_server" type="traj_server" output="log">
    <remap from="position_cmd" to="drone_{drone_id}_planning/pos_cmd"/>
    <remap from="~planning/bspline" to="drone_{drone_id}_planning/bspline"/>
    <param name="traj_server/time_forward" value="1.0" type="double"/>
  </node>

  <node pkg="poscmd_2_odom" name="drone_{drone_id}_poscmd_2_odom" type="poscmd_2_odom" output="log">
    <param name="init_x" value="{sx}"/>
    <param name="init_y" value="{sy}"/>
    <param name="init_z" value="{sz}"/>
    <remap from="~command" to="drone_{drone_id}_planning/pos_cmd"/>
    <remap from="~odometry" to="drone_{drone_id}_visual_slam/odom"/>
  </node>

  <node pkg="local_sensing_node" type="pcl_render_node" name="drone_{drone_id}_pcl_render_node" output="log">
    <param name="sensing_horizon" value="5.0"/>
    <param name="sensing_rate" value="30.0"/>
    <param name="estimation_rate" value="30.0"/>
    <param name="map/x_size" value="{PLANNER_MAP_X_M}"/>
    <param name="map/y_size" value="{PLANNER_MAP_Y_M}"/>
    <param name="map/z_size" value="{PLANNER_MAP_Z_M}"/>
    <remap from="~global_map" to="/map_generator/global_cloud"/>
    <remap from="~odometry" to="/drone_{drone_id}_visual_slam/odom"/>
    <remap from="~pcl_render_node/cloud" to="/drone_{drone_id}_pcl_render_node/cloud"/>
  </node>
"""
