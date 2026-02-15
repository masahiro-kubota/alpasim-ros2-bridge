# AlpaSim ROS2 Bridge Container
# Ubuntu 24.04 + ROS2 Jazzy + Python 3.12
#
# Build:
#   docker build -t alpasim-bridge:latest .
#
# Test (default):
#   docker run --rm alpasim-bridge:latest
#
# Run server:
#   docker run --rm --net=host alpasim-bridge:latest \
#     python3 -m alpasim_ros2_bridge.server

FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# uv をコピー（pip 不要）
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# 1. autoware_msgs を colcon build（.msg → Python クラス生成に必要）
COPY vendor/autoware_msgs/ /ros2_ws/src/autoware_msgs/
RUN . /opt/ros/jazzy/setup.sh && \
    cd /ros2_ws && \
    colcon build --packages-up-to \
        autoware_planning_msgs \
        autoware_perception_msgs \
        autoware_vehicle_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. venv 作成（--system-site-packages で rclpy 等の apt パッケージが見える）
RUN uv venv --system-site-packages --python python3.12 /opt/venv
ENV VIRTUAL_ENV=/opt/venv PATH="/opt/venv/bin:$PATH"

# 3. alpasim_grpc をインストール（.proto → _pb2.py コンパイルは build 時に自動実行）
COPY vendor/alpasim_grpc/ /tmp/alpasim_grpc/
RUN uv pip install /tmp/alpasim_grpc/ pytest pytest-asyncio

# 4. Bridge コードを配置（PYTHONPATH に置くだけ）
COPY alpasim_ros2_bridge/ /app/alpasim_ros2_bridge/
COPY tests/ /app/tests/
COPY config/ /app/config/

WORKDIR /app

ENTRYPOINT ["/bin/bash", "-c", \
    ". /opt/ros/jazzy/setup.sh && . /ros2_ws/install/setup.sh && exec \"$@\"", "--"]
CMD ["python3", "-m", "pytest", "tests/", "-v"]
