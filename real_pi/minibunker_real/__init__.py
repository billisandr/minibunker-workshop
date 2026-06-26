"""Native (no-ROS) MiniBunker stack for the Raspberry Pi.

Modules:
    config           dotted-key YAML config loader
    detector         HSV / CNN detectors (lifted from the ROS detector_node)
    perception_state role-based 7-slot packing shared sim<->real
    fsm              reactive behaviour state machine -> (linear, angular)
    camera           picamera2 / V4L2 / video / synthetic frame source
    bunker_can       AgileX protocol-v2 CAN driver for the Bunker Mini base
"""
