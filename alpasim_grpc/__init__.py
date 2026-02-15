"""AlpaSim gRPC definitions (bridge-local copy)."""

__version__ = (0, 53, 0)

from alpasim_grpc.v0.common_pb2 import VersionId

API_VERSION_MESSAGE = VersionId.APIVersion(
    major=__version__[0],
    minor=__version__[1],
    patch=__version__[2],
)
