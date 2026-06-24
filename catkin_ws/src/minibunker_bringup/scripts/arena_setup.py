#!/usr/bin/env python3
"""One-shot node: spawns the minibunker arena's fence + cones in Gazebo, driven
by config (arena/fence_enabled, arena/cone_scale, see minibunker.yaml). Models
are spawned one at a time via /gazebo/spawn_sdf_model rather than baked into
the static .world file, for two reasons: (1) it keeps fence presence and cone
size config-driven without editing SDF, and (2) spawning sequentially avoids a
real Gazebo/Ogre race seen when several copies of the same mesh+texture
(construction_cone) load concurrently off a single static <include>."""
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

FENCE_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="fence">
    <static>true</static>
    <link name="link">
      <collision name="n"><pose>0 3 0.25 0 0 0</pose><geometry><box><size>6 0.1 0.5</size></box></geometry></collision>
      <visual name="nv"><pose>0 3 0.25 0 0 0</pose><geometry><box><size>6 0.1 0.5</size></box></geometry><material><ambient>0.5 0.4 0.1 1</ambient><diffuse>0.6 0.45 0.12 1</diffuse></material></visual>
      <collision name="s"><pose>0 -3 0.25 0 0 0</pose><geometry><box><size>6 0.1 0.5</size></box></geometry></collision>
      <visual name="sv"><pose>0 -3 0.25 0 0 0</pose><geometry><box><size>6 0.1 0.5</size></box></geometry><material><ambient>0.5 0.4 0.1 1</ambient><diffuse>0.6 0.45 0.12 1</diffuse></material></visual>
      <collision name="e"><pose>3 0 0.25 0 0 0</pose><geometry><box><size>0.1 6 0.5</size></box></geometry></collision>
      <visual name="ev"><pose>3 0 0.25 0 0 0</pose><geometry><box><size>0.1 6 0.5</size></box></geometry><material><ambient>0.5 0.4 0.1 1</ambient><diffuse>0.6 0.45 0.12 1</diffuse></material></visual>
      <collision name="w"><pose>-3 0 0.25 0 0 0</pose><geometry><box><size>0.1 6 0.5</size></box></geometry></collision>
      <visual name="wv"><pose>-3 0 0.25 0 0 0</pose><geometry><box><size>0.1 6 0.5</size></box></geometry><material><ambient>0.5 0.4 0.1 1</ambient><diffuse>0.6 0.45 0.12 1</diffuse></material></visual>
    </link>
  </model>
</sdf>
"""

# Upstream construction_cone/model.sdf (vendored under ~/.gazebo/models, see
# docker/Dockerfile) scales its mesh by a flat 10x; {scale} multiplies that
# further by arena/cone_scale so the include stays a single uniform knob.
CONE_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.5">
  <model name="construction_cone">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <scale>{scale} {scale} {scale}</scale>
            <uri>model://construction_cone/meshes/construction_cone.dae</uri>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <scale>{scale} {scale} {scale}</scale>
            <uri>model://construction_cone/meshes/construction_cone.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""

CONE_POSITIONS = {
    "cone_1": (1.4, -0.9),
    "cone_2": (2.0, 1.1),
    "cone_3": (3.2, -1.4),
}


def _pose(x, y):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, 0.0
    return p


def main():
    rospy.init_node("arena_setup", anonymous=False)

    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    if rospy.get_param("arena/fence_enabled", False):
        resp = spawn("fence", FENCE_SDF, "", Pose(), "world")
        rospy.loginfo("[arena_setup] fence spawn: %s", resp.status_message)
    else:
        rospy.loginfo("[arena_setup] fence_enabled=false -> not spawning the fence")

    scale = 10.0 * float(rospy.get_param("arena/cone_scale", 1.0))
    cone_sdf = CONE_SDF_TEMPLATE.format(scale=scale)
    for name, (x, y) in CONE_POSITIONS.items():
        resp = spawn(name, cone_sdf, "", _pose(x, y), "world")
        rospy.loginfo("[arena_setup] %s spawn: %s", name, resp.status_message)


if __name__ == "__main__":
    main()
