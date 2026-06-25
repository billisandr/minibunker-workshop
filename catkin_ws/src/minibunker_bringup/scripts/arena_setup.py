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
#
# The cone is DYNAMIC (was <static>true</static>, an immovable brick) so the
# heavy rover knocks it aside. Two deliberate choices:
#  - COLLISION is a cheap CYLINDER, not the mesh: dynamic mesh-mesh collisions in
#    ODE are slow and jittery; a primitive makes the knock-over stable. The
#    detailed mesh stays as the (orange) VISUAL the detector sees.
#  - friction mu is HIGHER than the ball's (it resists rolling more), but it
#    still skids when hit. Inertia matches the cylinder so it tips/skids sanely.
CONE_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.5">
  <model name="construction_cone">
    <static>false</static>
    <link name="link">
      <inertial>
        <pose>0 0 {cz} 0 0 0</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 {cz} 0 0 0</pose>
        <geometry>
          <cylinder><radius>{cr}</radius><length>{ch}</length></cylinder>
        </geometry>
        <surface>
          <friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction>
          <contact><ode><kp>100000.0</kp><kd>1.0</kd></ode></contact>
        </surface>
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
    # Approx physical size of the scaled cone mesh (real ~0.45 m tall at scale 10).
    cone_h = 0.045 * scale
    cone_r = 0.014 * scale
    mass = float(rospy.get_param("arena/cone_mass", 0.4))   # > ball: more resistance
    mu = float(rospy.get_param("arena/cone_mu", 0.5))       # > ball's 0.2
    cz = cone_h / 2.0
    izz = 0.5 * mass * cone_r ** 2
    ixx = iyy = (1.0 / 12.0) * mass * (3.0 * cone_r ** 2 + cone_h ** 2)
    cone_sdf = CONE_SDF_TEMPLATE.format(
        scale=scale, mass=mass, mu=mu, cr=cone_r, ch=cone_h, cz=cz,
        ixx=ixx, iyy=iyy, izz=izz)
    for name, (x, y) in CONE_POSITIONS.items():
        resp = spawn(name, cone_sdf, "", _pose(x, y), "world")
        rospy.loginfo("[arena_setup] %s spawn: %s", name, resp.status_message)


if __name__ == "__main__":
    main()
