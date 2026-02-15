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

# 1. Install autoware message packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-autoware-planning-msgs \
        ros-jazzy-autoware-perception-msgs \
        ros-jazzy-autoware-vehicle-msgs \
    && rm -rf /var/lib/apt/lists/*

# 2. Install uv
COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# 3. Create venv (--system-site-packages to access rclpy and apt packages)
RUN uv venv --system-site-packages --python python3.12 /opt/venv
ENV VIRTUAL_ENV=/opt/venv PATH="/opt/venv/bin:$PATH"

# 4. Install Python dependencies
RUN uv pip install grpcio grpcio-tools "protobuf>=4.0.0,<5.0.0" \
    dataclasses-json numpy pytest pytest-asyncio

# 5. Copy proto definitions and compile (.proto -> _pb2.py)
COPY alpasim_grpc/ /app/alpasim_grpc/
RUN cd /app && python -c \
    "from grpc_tools.command import build_package_protos; build_package_protos('.', strict_mode=True)"

# 6. Place bridge code on PYTHONPATH
COPY alpasim_ros2_bridge/ /app/alpasim_ros2_bridge/
COPY tests/ /app/tests/
COPY config/ /app/config/

WORKDIR /app

ENTRYPOINT ["/bin/bash", "-c", \
    ". /opt/ros/jazzy/setup.sh && exec \"$@\"", "--"]
CMD ["python3", "-m", "pytest", "tests/", "-v"]
