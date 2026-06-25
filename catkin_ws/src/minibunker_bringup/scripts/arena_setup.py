#!/usr/bin/env python3
"""Spawn EVERY arena object from the single `arena/objects` block in
minibunker.yaml — the one place all props are defined. The .world file holds
only the ground plane, lighting and physics; this node builds an SDF for each
object from its yaml spec (type, size, pose, static, colour, mass, friction mu,
contact kp/kd) and spawns it via /gazebo/spawn_sdf_model.

Inertia is AUTO-COMPUTED from type+size+mass so it is always physically
consistent (a mismatched inertia fights motion); override per object with an
explicit `inertia: [ixx, iyy, izz]` only if you must.

Objects are spawned one at a time (sequential service calls) to dodge a real
Gazebo/Ogre race seen when several copies of the same cone mesh+texture load at
once. See docs/ARENA.md for the schema.
"""
import math

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

CONE_MESH = "model://construction_cone/meshes/construction_cone.dae"


# --- pose ------------------------------------------------------------------
def _pose(x, y, z, yaw=0.0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.z = math.sin(yaw / 2.0)
    p.orientation.w = math.cos(yaw / 2.0)
    return p


# --- geometry: size, inertia + the cone-mesh cylinder approximation --------
def _cyl_dims(otype, spec):
    if otype == "cylinder":
        return float(spec["radius"]), float(spec["length"])
    # cone_mesh: the scaled construction cone (~0.45 m tall at scale 10),
    # approximated by a cylinder for stable, cheap dynamic collision.
    s = float(spec.get("scale", 3.0))
    return 0.014 * s, 0.045 * s


def _inertia(otype, spec, mass):
    """(ixx, iyy, izz, com_z) for the geometry, COM resting on the floor."""
    if otype == "sphere":
        r = float(spec["radius"])
        i = 0.4 * mass * r * r              # solid sphere: 2/5 m r^2
        return i, i, i, r
    if otype == "box":
        a, b, c = (float(v) for v in spec["size"])
        return ((1 / 12.0) * mass * (b * b + c * c),
                (1 / 12.0) * mass * (a * a + c * c),
                (1 / 12.0) * mass * (a * a + b * b),
                c / 2.0)
    if otype in ("cylinder", "cone_mesh"):
        r, h = _cyl_dims(otype, spec)
        return ((1 / 12.0) * mass * (3 * r * r + h * h),
                (1 / 12.0) * mass * (3 * r * r + h * h),
                0.5 * mass * r * r,
                h / 2.0)
    return 0.0, 0.0, 0.0, 0.0


# --- SDF fragments ---------------------------------------------------------
def _material(color):
    if not color:
        return ""
    r, g, b = (float(c) for c in color)
    return ("<material><ambient>%g %g %g 1</ambient>"
            "<diffuse>%g %g %g 1</diffuse></material>" % (r, g, b, r, g, b))


def _surface(mu, kp, kd):
    return ("<surface>"
            "<friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction>"
            "<contact><ode><kp>%g</kp><kd>%g</kd></ode></contact></surface>"
            % (mu, mu, kp, kd))


def _inertial(mass, ixx, iyy, izz, com_z):
    return ("<inertial><pose>0 0 %g 0 0 0</pose><mass>%g</mass><inertia>"
            "<ixx>%g</ixx><iyy>%g</iyy><izz>%g</izz>"
            "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
            % (com_z, mass, ixx, iyy, izz))


def _geom(otype, spec):
    if otype == "sphere":
        return "<sphere><radius>%g</radius></sphere>" % float(spec["radius"])
    if otype == "box":
        a, b, c = (float(v) for v in spec["size"])
        return "<box><size>%g %g %g</size></box>" % (a, b, c)
    if otype == "plane":
        s = float(spec.get("size", 12.0))
        return "<plane><normal>0 0 1</normal><size>%g %g</size></plane>" % (s, s)
    if otype in ("cylinder", "cone_mesh"):
        r, h = _cyl_dims(otype, spec)
        return "<cylinder><radius>%g</radius><length>%g</length></cylinder>" % (r, h)
    raise ValueError("unknown geometry type: %s" % otype)


def _model_sdf(name, static, inner):
    return ('<?xml version="1.0"?><sdf version="1.6"><model name="%s">'
            '<static>%s</static><link name="link">%s</link></model></sdf>'
            % (name, "true" if static else "false", inner))


# --- per-object builders ---------------------------------------------------
def _prop_sdf(name, spec):
    """Build a single prop (sphere/box/cylinder/cone_mesh)."""
    otype = spec["type"]
    static = bool(spec.get("static", False))
    mu = float(spec.get("mu", 0.5))
    kp = float(spec.get("kp", 1000.0))
    kd = float(spec.get("kd", 1.0))
    color = spec.get("color")

    _, _, _, gz = _inertia(otype, spec, 1.0)   # geometry centre height
    col = ('<collision name="c"><pose>0 0 %g 0 0 0</pose><geometry>%s</geometry>'
           '%s</collision>' % (gz, _geom(otype, spec), _surface(mu, kp, kd)))

    if otype == "cone_mesh":
        # textured mesh visual sits on its own base; no material/colour.
        s = float(spec.get("scale", 3.0))
        vis = ('<visual name="v"><geometry><mesh><scale>%g %g %g</scale>'
               '<uri>%s</uri></mesh></geometry></visual>' % (s, s, s, CONE_MESH))
    else:
        vis = ('<visual name="v"><pose>0 0 %g 0 0 0</pose><geometry>%s</geometry>'
               '%s</visual>' % (gz, _geom(otype, spec), _material(color)))

    inner = col + vis
    if not static:
        mass = float(spec.get("mass", 1.0))
        ixx, iyy, izz, com_z = _inertia(otype, spec, mass)
        override = spec.get("inertia")
        if override:
            ixx, iyy, izz = (float(v) for v in override)
        inner = _inertial(mass, ixx, iyy, izz, com_z) + inner
    return _model_sdf(name, static, inner)


def _light_sdf(name, spec):
    """A world light (directional 'sun', or point/spot). Position is cosmetic for
    a directional light — only <direction> matters."""
    lt = str(spec.get("light_type", "directional"))
    d = spec.get("direction", [-0.4, 0.3, -0.9])
    diff = spec.get("diffuse", [0.9, 0.9, 0.9])
    spc = spec.get("specular", [0.2, 0.2, 0.2])
    cast = "true" if spec.get("cast_shadows", True) else "false"
    return ('<?xml version="1.0"?><sdf version="1.6">'
            '<light type="%s" name="%s"><pose>0 0 10 0 0 0</pose>'
            '<cast_shadows>%s</cast_shadows>'
            '<diffuse>%g %g %g 1</diffuse><specular>%g %g %g 1</specular>'
            '<direction>%g %g %g</direction></light></sdf>'
            % (lt, name, cast, diff[0], diff[1], diff[2],
               spc[0], spc[1], spc[2], d[0], d[1], d[2]))


def _fence_sdf(name, spec):
    """A four-wall boundary ring (static)."""
    span = float(spec.get("span", 6.0))
    h = float(spec.get("height", 0.5))
    t = float(spec.get("thickness", 0.1))
    half, cz = span / 2.0, h / 2.0
    mat = _material(spec.get("color", [0.6, 0.45, 0.12]))
    walls = ""
    for tag, wx, wy, sx, sy in (("n", 0, half, span, t), ("s", 0, -half, span, t),
                                ("e", half, 0, t, span), ("w", -half, 0, t, span)):
        box = "<box><size>%g %g %g</size></box>" % (sx, sy, h)
        walls += ('<collision name="%sc"><pose>%g %g %g 0 0 0</pose>'
                  '<geometry>%s</geometry></collision>'
                  '<visual name="%sv"><pose>%g %g %g 0 0 0</pose>'
                  '<geometry>%s</geometry>%s</visual>'
                  % (tag, wx, wy, cz, box, tag, wx, wy, cz, box, mat))
    return _model_sdf(name, True, walls)


def _build_sdf(name, spec):
    t = spec.get("type")
    if t == "fence":
        return _fence_sdf(name, spec)
    if t == "light":
        return _light_sdf(name, spec)
    return _prop_sdf(name, spec)


# --- main ------------------------------------------------------------------
def main():
    rospy.init_node("arena_setup", anonymous=False)
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=60.0)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    objects = rospy.get_param("arena/objects", {})
    if not objects:
        rospy.logwarn("[arena_setup] no arena/objects in config — nothing to spawn")
        return

    # Spawn the floor + lights first so props always have ground to rest on.
    def _order(item):
        return 0 if item[1].get("type") in ("plane", "light") else 1

    for name, spec in sorted(objects.items(), key=_order):
        if not spec.get("enabled", True):
            rospy.loginfo("[arena_setup] %s disabled — skipping", name)
            continue
        otype = spec.get("type")
        try:
            sdf = _build_sdf(name, spec)
        except (KeyError, ValueError) as exc:
            rospy.logwarn("[arena_setup] %s spec invalid (%s) — skipping", name, exc)
            continue
        if otype == "light":
            pose = _pose(0.0, 0.0, 0.0)        # directional: position is cosmetic
        else:
            p = spec.get("pose", [0.0, 0.0])
            yaw = float(p[2]) if len(p) > 2 else 0.0
            pose = _pose(p[0], p[1], 0.0, yaw)
        resp = spawn(name, sdf, "", pose, "world")
        rospy.loginfo("[arena_setup] %s (%s): %s", name, otype, resp.status_message)


if __name__ == "__main__":
    main()
