"""RealSense colour (and optional close-range depth) reader for the B601 wrist camera."""

from __future__ import annotations

import sys
from collections.abc import Callable
from typing import Any

import numpy as np
import numpy.typing as npt

Image = npt.NDArray[np.uint8]  # height x width x 3 RGB
Depth = npt.NDArray[np.float64]  # height x width metres, 0 for no return

# RealSense Enhanced Depth package (librealsense2-enhanced-depth .deb, NVIDIA Jetson only).
# Used when installed; otherwise depth comes straight from the camera.
ENHANCED_DEPTH_PATH = "/opt/librealsense2-enhanced-depth/python"


def _close_range_improver(profile: Any, rs: Any) -> tuple[Any, str | None]:
    """The close-range DepthRangeImprover for this stream, or (None, reason)."""
    if ENHANCED_DEPTH_PATH not in sys.path:
        sys.path.append(ENHANCED_DEPTH_PATH)
    try:
        from rs_depth import Calibration, DepthRangeImprover, discover_backends
        if not discover_backends().has_depth_range:
            return None, "close-range backend not installed"
        left = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        right = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        return DepthRangeImprover(Calibration.from_sdk(left.get_intrinsics(),
                                                       left.get_extrinsics_to(right))), None
    except Exception as error:
        return None, f"close-range depth unavailable: {error}"


class RealSenseCamera:
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30, depth: bool = False) -> None:
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        if min(self.width, self.height) <= 0 or self.fps <= 0:
            raise ValueError("Camera dimensions and fps must be positive")
        self.depth = bool(depth)
        self.depth_backend: str | None = None  # "close-range" or "hardware" once started with depth
        self._pipeline: Any = None
        self._improver: Any = None
        self._depth_scale: float | None = None
        # Where status lines go; the embodiment points this at its operator session.
        self.notify: Callable[[str], None] = lambda text: print(text, flush=True)

    def start(self) -> RealSenseCamera:
        if self._pipeline is not None:
            return self
        import pyrealsense2 as rs

        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height,
                             rs.format.bgr8, self.fps)
        if self.depth:
            # The D405 takes colour from the left imager, so depth is already pixel-aligned with it.
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
            config.enable_stream(rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps)
        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        self._pipeline = pipeline
        if self.depth:
            self._depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self._improver, reason = _close_range_improver(profile, rs)
            self.depth_backend = "close-range" if self._improver is not None else "hardware"
            if reason:
                self.notify(f"camera depth: {reason}; using hardware depth only")
            self.read_with_depth()  # Warm up the improver while auto-exposure settles.
        else:
            self.read()  # Drop the first frame while auto-exposure settles.
        return self

    def _frames(self) -> tuple[Any, Image]:
        if self._pipeline is None:
            raise RuntimeError("camera is not started")
        frames = self._pipeline.wait_for_frames(5000)
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("RealSense returned no color frame")
        image: Image = np.asanyarray(color.get_data())[:, :, ::-1].copy()
        if image.shape != (self.height, self.width, 3) or image.dtype != np.uint8:
            raise RuntimeError(f"Unexpected RealSense frame {image.shape} {image.dtype}")
        return frames, image

    def read(self) -> Image:
        return self._frames()[1]

    def read_with_depth(self) -> tuple[Image, Depth]:
        """(RGB image, metric depth in metres with 0 for no return) from one synchronized frameset."""
        if not self.depth:
            raise RuntimeError("camera was not configured with depth")
        frames, image = self._frames()
        if self._depth_scale is None:
            raise RuntimeError("camera depth scale is unknown")
        depth_mm = (np.asanyarray(frames.get_depth_frame().get_data()).astype(np.float64)
                    * (self._depth_scale * 1000.0))
        if self._improver is not None:
            left = np.asanyarray(frames.get_infrared_frame(1).get_data())
            right = np.asanyarray(frames.get_infrared_frame(2).get_data())
            depth_mm = self._improver.process(left, right, depth_mm.round().astype(np.uint16))
        return image, np.asarray(depth_mm, dtype=np.float64) / 1000.0

    def close(self) -> None:
        pipeline, self._pipeline = self._pipeline, None
        self._improver = None
        if pipeline is not None:
            pipeline.stop()


def parse_camera(value: object) -> RealSenseCamera | None:
    """Parse ``realsense[+depth]`` or ``realsense[+depth]:WIDTHxHEIGHT@FPS``; empty disables."""
    if not value:
        return None
    parts = str(value).split(":", 1)
    if parts[0] not in ("realsense", "realsense+depth"):
        raise ValueError("Only camera=realsense[+depth][:WxH@FPS] is supported")
    depth = parts[0] == "realsense+depth"
    if len(parts) == 1:
        return RealSenseCamera(depth=depth)
    resolution, _, fps = parts[1].partition("@")
    width, height = (int(item) for item in resolution.lower().split("x"))
    return RealSenseCamera(width, height, int(fps or 30), depth=depth)
