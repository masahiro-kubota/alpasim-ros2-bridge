# AlpaSim ROS2 Bridge

A gRPC-based bridge that implements AlpaSim's [EgodriverService](https://github.com/nvidia-cosmos/alpasim), enabling ROS2 planners to drive closed-loop simulations.

## How it works

The bridge implements AlpaSim's standard `EgodriverService` interface, acting as a drop-in replacement for the driver service. AlpaSim calls `submit_image_observation()`, `submit_egomotion_observation()`, and `submit_route()` to send data, then calls `drive()` which publishes cached data to ROS2 topics, waits for the planner's trajectory response on `/planning/trajectory`, and returns it to AlpaSim.

**No AlpaSim code changes required** — simply deploy the bridge container with the service name `driver`.

## Build & Test

Everything runs inside Docker (ROS2 Jazzy + Python 3.12):

```bash
# Build
docker build -t alpasim-bridge:latest .

# Run tests (default CMD)
docker run --rm alpasim-bridge:latest

# Run the bridge server (as "driver" service)
docker run --rm --net=host --name driver alpasim-bridge:latest \
  python3 -m alpasim_ros2_bridge.server
```

## Integration with AlpaSim

Start the bridge container with service name **`driver`**:

```bash
docker run --rm --net=host --name driver alpasim-bridge:latest \
  python3 -m alpasim_ros2_bridge.server
```

Then start AlpaSim normally. It will automatically connect to the bridge via the `driver:50060` endpoint. No configuration changes needed.

## Proto files

The `alpasim_grpc/` directory contains `.proto` files copied from AlpaSim (`src/grpc/alpasim_grpc/v0/`):
- `common.proto` - Common data types (Pose, Trajectory, DynamicState)
- `egodriver.proto` - EgodriverService interface
- `sensorsim.proto` - Camera specifications (needed for image resolution)

These are compiled to `_pb2.py` at Docker build time via `grpcio-tools`. If the AlpaSim gRPC API changes, update these files manually.

## ROS2 Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/clock` | `rosgraph_msgs/Clock` | Publish |
| `/tf` | `tf2_msgs/TFMessage` | Publish |
| `/camera/{name}/image_raw` | `sensor_msgs/Image` | Publish |
| `/camera/{name}/camera_info` | `sensor_msgs/CameraInfo` | Publish |
| `/vehicle/status/velocity` | `autoware_vehicle_msgs/VelocityReport` | Publish |
| `/planning/trajectory` | `autoware_planning_msgs/Trajectory` | Subscribe |

**Note**: The bridge caches data from `submit_*()` calls and publishes to ROS2 when `drive()` is called, ensuring proper synchronization.

## Architecture

```
AlpaSim Container
  └─ loop.py → driver (EgodriverService)
                 ↓ gRPC
Bridge Container ("driver" name)
  └─ EgodriverServicer
       ├─ submit_*() → cache data
       └─ drive() → publish to ROS2 & wait for trajectory
                 ↓ ROS2 Topics
Planner Container
  └─ ROS2 Planner Node
       └─ /planning/trajectory publisher
```

## License

Apache-2.0
