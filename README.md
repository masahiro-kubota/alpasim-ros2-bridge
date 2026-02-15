# AlpaSim ROS2 Bridge

A gRPC-based bridge that connects [AlpaSim](https://github.com/nvidia-cosmos/alpasim) to the ROS2 ecosystem, enabling ROS2 planners to drive closed-loop simulations.

## How it works

AlpaSim calls `bridge.step()` via gRPC each simulation tick. The bridge publishes sensor data, TF, and perception info to ROS2 topics, waits for a planner's trajectory response, and returns it to AlpaSim.

## Build & Test

Everything runs inside Docker (ROS2 Jazzy + Python 3.12):

```bash
# Build
docker build -t alpasim-bridge:latest .

# Run tests (default CMD)
docker run --rm alpasim-bridge:latest

# Run the bridge server
docker run --rm --net=host alpasim-bridge:latest \
  python3 -m alpasim_ros2_bridge.server
```

## Proto files

The `alpasim_grpc/` directory contains `.proto` files copied from the AlpaSim repository (`src/grpc/alpasim_grpc/v0/`). These are compiled to `_pb2.py` at Docker build time via `grpcio-tools`. If the AlpaSim gRPC API changes, update these files manually.

## ROS2 Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/clock` | `rosgraph_msgs/Clock` | Publish |
| `/tf` | `tf2_msgs/TFMessage` | Publish |
| `/camera/{name}/image_raw` | `sensor_msgs/Image` | Publish |
| `/perception/objects` | `autoware_perception_msgs/TrackedObjects` | Publish |
| `/vehicle/status/velocity` | `autoware_vehicle_msgs/VelocityReport` | Publish |
| `/planning/trajectory` | `autoware_planning_msgs/Trajectory` | Subscribe |

## License

Apache-2.0
