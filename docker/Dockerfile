# ============================================================================
#  MiniBunker Workshop — ROS1 Noetic image (sim + real)
#
#  One Dockerfile, two targets (plan.md §11):
#    • SIM  (x86 dev laptop): default base osrf/ros:noetic-desktop-full ships
#      Gazebo + RViz; build the whole catkin_ws and run the sim.
#    • REAL (Raspberry Pi 5, arm64): build with
#         --build-arg ROS_BASE_IMAGE=arm64v8/ros:noetic-ros-base
#      and bring up the Pi camera + CAN -> bunker_base. gazebo isn't needed
#      on the Pi, but the C++ CAN driver (ugv_sdk/bunker_base) still compiles
#      fine off-hardware (it only needs the CAN netdev at *runtime*).
#
#  Build context is the repo root: `docker build -f docker/Dockerfile .`
# ============================================================================
ARG ROS_BASE_IMAGE=osrf/ros:noetic-desktop-full
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System + ROS dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # build / tooling
    build-essential cmake git nano tmux wget curl \
    iputils-ping net-tools psmisc \
    python3-pip python3-rosdep \
    # ugv_sdk / bunker_base C++ build deps
    libasio-dev libboost-all-dev libeigen3-dev \
    # CAN bring-up + python CAN (real robot)
    can-utils iproute2 \
    # ROS: gazebo + control (sim); harmless to install on arm if present
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    ros-noetic-gazebo-dev ros-noetic-gazebo-msgs ros-noetic-gazebo-plugins \
    ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
    ros-noetic-controller-manager ros-noetic-joint-state-controller \
    ros-noetic-position-controllers ros-noetic-effort-controllers \
    ros-noetic-xacro \
    # roslint: build-time catkin component required by the vendored
    # bunker_gazebo_sim package; NOT pulled in by desktop-full, so without this
    # `find_package(catkin REQUIRED COMPONENTS roslint)` fails the build.
    ros-noetic-roslint \
    # ROS: perception + bridge
    ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-vision-msgs \
    ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
    ros-noetic-rosbridge-suite \
    # ROS: viz / debugging (no-ops on ros-base if unavailable -> see note)
    ros-noetic-rqt-image-view ros-noetic-image-view \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Python perception runtime deps
#   - numpy/opencv come from python3-opencv (system, cv_bridge-compatible)
#   - onnxruntime: default CNN runtime, has both amd64 + arm64 wheels
#   - python-can: real-robot CAN fallback path (plan §9.4)
#   ultralytics (.pt) and ncnn are OPTIONAL and heavy; install them only when
#   needed via the build args below (keep the default image lean for the Pi).
# ---------------------------------------------------------------------------
ARG INSTALL_ONNXRUNTIME=true
ARG INSTALL_NCNN=false
ARG INSTALL_ULTRALYTICS=false
RUN if [ "$INSTALL_ONNXRUNTIME" = "true" ]; then pip3 install --no-cache-dir onnxruntime; fi \
 && if [ "$INSTALL_NCNN" = "true" ];        then pip3 install --no-cache-dir ncnn; fi \
 && if [ "$INSTALL_ULTRALYTICS" = "true" ]; then pip3 install --no-cache-dir ultralytics; fi \
 && pip3 install --no-cache-dir python-can

# Fail the build loudly if the perception trio can't coexist with cv_bridge —
# the single highest-risk integration point (plan.md §11, same class of bug as
# the opencv/protobuf clash in ros_z1_teleop).
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    python3 -c 'import cv2, numpy; from cv_bridge import CvBridge; \
print(\"smoke test OK: cv2\", cv2.__version__, \"| numpy\", numpy.__version__)'"

# ---------------------------------------------------------------------------
# Non-root user
# ---------------------------------------------------------------------------
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG video,dialout $USERNAME

# ---------------------------------------------------------------------------
# Copy the catkin workspace (our 4 packages + 3 vendored AgileX submodules)
# ---------------------------------------------------------------------------
RUN mkdir -p /home/$USERNAME/catkin_ws
COPY --chown=$USERNAME:$USERNAME catkin_ws/src /home/$USERNAME/catkin_ws/src
# The master config lives inside minibunker_bringup/config (copied with src),
# so launch files resolve it via $(find minibunker_bringup)/config/minibunker.yaml.

# Vendored submodules can ship CRLF line endings (Windows clones) that break
# $-anchored sed/cmake parsing — normalise defensively (same fix class as the
# z1_controller CMakeLists in ros_z1_teleop). Harmless if already LF.
RUN find /home/$USERNAME/catkin_ws/src -name 'CMakeLists.txt' -exec sed -i 's/\r$//' {} +

# The catkin_ws dir itself was created by `mkdir` as root; catkin_make (run as
# $USERNAME below) needs to create build/ and devel/ inside it, so hand the
# whole tree to the user before switching. Without this: catkin_make fails with
# "[Errno 13] Permission denied: '/home/rosuser/catkin_ws/build'".
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/catkin_ws

USER $USERNAME
ENV HOME=/home/$USERNAME

# ---------------------------------------------------------------------------
# Vendor the "construction_cone" Gazebo model (mirrors models.gazebosim.org,
# fetched from the OSRF GitHub mirror at build time) into ~/.gazebo/models so
# the minibunker world's <include><uri>model://construction_cone</uri></include>
# resolves with zero network access at sim runtime -- same "runs offline"
# requirement as the rest of minibunker_arena.world.
# ---------------------------------------------------------------------------
RUN mkdir -p $HOME/.gazebo/models/construction_cone/meshes \
             $HOME/.gazebo/models/construction_cone/materials/textures \
 && BASE=https://raw.githubusercontent.com/osrf/gazebo_models/master/construction_cone \
 && curl -sSL -o $HOME/.gazebo/models/construction_cone/model.config        "$BASE/model.config" \
 && curl -sSL -o $HOME/.gazebo/models/construction_cone/model.sdf          "$BASE/model.sdf" \
 && curl -sSL -o $HOME/.gazebo/models/construction_cone/meshes/construction_cone.dae \
        "$BASE/meshes/construction_cone.dae" \
 && curl -sSL -o $HOME/.gazebo/models/construction_cone/materials/textures/Construction_Cone_Diffuse.png \
        "$BASE/materials/textures/Construction_Cone_Diffuse.png"

# ---------------------------------------------------------------------------
# Build the workspace
#   ugv_sdk -> bunker_msgs -> bunker_base, plus our minibunker_* packages and
#   the Bunker gazebo description. catkin resolves the build order.
# ---------------------------------------------------------------------------
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd \$HOME/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release"

# ---------------------------------------------------------------------------
# Shell setup: auto-source + workshop aliases
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash"            >> $HOME/.bashrc && \
    echo "source \$HOME/catkin_ws/devel/setup.bash"     >> $HOME/.bashrc && \
    echo 'export PS1="\[\033[01;36m\]\u@minibunker\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> $HOME/.bashrc && \
    echo '# --- Station launches ---'                   >> $HOME/.bashrc && \
    echo 'alias mb_sim="roslaunch minibunker_bringup minibunker_sim.launch"'   >> $HOME/.bashrc && \
    echo 'alias mb_real="roslaunch minibunker_bringup minibunker_real.launch"' >> $HOME/.bashrc && \
    echo 'alias mb_unpause="rosservice call /gazebo/unpause_physics"'          >> $HOME/.bashrc && \
    echo '# --- Individual nodes ---'                    >> $HOME/.bashrc && \
    echo 'alias mb_detector="rosrun minibunker_perception detector_node.py"'   >> $HOME/.bashrc && \
    echo 'alias mb_behavior="rosrun minibunker_behavior behavior_node.py"'     >> $HOME/.bashrc && \
    echo '# --- Diagnostics ---'                         >> $HOME/.bashrc && \
    echo 'alias mb_cam="rosrun image_view image_view image:=/minibunker/debug_image"' >> $HOME/.bashrc && \
    echo 'alias mb_arm="rostopic pub -1 /minibunker/arm std_msgs/Bool \"data: true\""'    >> $HOME/.bashrc && \
    echo 'alias mb_disarm="rostopic pub -1 /minibunker/arm std_msgs/Bool \"data: false\""' >> $HOME/.bashrc

# GUI / GPU env (sim on the dev laptop — same llvmpipe story as ros_z1_teleop;
# harmless on the headless Pi).
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3

WORKDIR $HOME
COPY --chown=$USERNAME:$USERNAME docker/entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x /home/$USERNAME/entrypoint.sh
ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
CMD ["roslaunch", "minibunker_bringup", "minibunker_sim.launch"]
