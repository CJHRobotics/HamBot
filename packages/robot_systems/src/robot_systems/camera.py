from abc import ABC, abstractmethod
import threading
import numpy as np
import cv2
from robot_systems.landmark import Landmark


class Camera(ABC):
    """Abstract base class for all HamBot camera backends.

    Provides shared landmark detection, color targeting, and frame management.
    Use Camera.create() to instantiate the appropriate subclass.
    """

    CAM_PICAM = 'PICAM'
    CAM_OAKD  = 'OAKD'

    @classmethod
    def create(cls, camera_type, **kwargs):
        """Factory method — return the correct Camera subclass for camera_type.

        Args:
            camera_type (str): Camera.CAM_PICAM or Camera.CAM_OAKD.
            **kwargs: Passed directly to the subclass constructor.

        Returns:
            Camera: An initialised PiCamera or OakCamera instance.

        Raises:
            ValueError: If camera_type is not recognised.
        """
        from robot_systems.pi_camera import PiCamera
        from robot_systems.oak_camera import OakCamera

        if camera_type == cls.CAM_PICAM:
            return PiCamera(**kwargs)
        elif camera_type == cls.CAM_OAKD:
            return OakCamera(**kwargs)
        else:
            raise ValueError(
                f"Unknown camera_type '{camera_type}'. "
                f"Use Camera.CAM_PICAM or Camera.CAM_OAKD."
            )

    def __init__(self, fps=5, rotate_180=False, tolerance=0.05):
        """Initialise shared camera state.

        Args:
            fps (int): Target capture frame rate. Defaults to 5.
            rotate_180 (bool): Rotate frame 180° to correct for upside-down
                               mounting. Defaults to False.
            tolerance (float): Fractional color tolerance for landmark
                               detection in [0, 1]. Defaults to 0.05.
        """
        self._fps            = max(1, int(fps))
        self._rotate_180     = bool(rotate_180)
        self._frame_rgb      = None
        self._frame_lock     = threading.Lock()
        self._target_colors_rgb = []
        self._color_tolerance   = float(np.clip(tolerance, 0.0, 1.0))
        self._running        = True

    @abstractmethod
    def get_frame(self, copy=True):
        """Return the latest RGB frame.

        Args:
            copy (bool): If True return a copy; otherwise the internal array.

        Returns:
            numpy.ndarray | None: (H, W, 3) uint8 RGB array, or None if no
                                  frame has been captured yet.
        """

    @abstractmethod
    def stop_camera(self):
        """Stop capture and release all hardware resources."""

    def find_landmarks(self, min_area=500):
        """Detect regions matching any configured target color.

        Args:
            min_area (int): Minimum contour area in pixels to report.
                            Defaults to 500.

        Returns:
            list[Landmark]: Detected landmarks sorted by area (largest first).
        """
        frame = self.get_frame(copy=False)
        if frame is None or not self._target_colors_rgb:
            return []

        tol  = int(round(255 * self._color_tolerance))
        h, w = frame.shape[:2]
        mask_total = np.zeros((h, w), dtype=np.uint8)

        for (r, g, b) in self._target_colors_rgb:
            lower = np.array([max(0, r - tol), max(0, g - tol), max(0, b - tol)], dtype=np.uint8)
            upper = np.array([min(255, r + tol), min(255, g + tol), min(255, b + tol)], dtype=np.uint8)
            mask_total = cv2.bitwise_or(mask_total, cv2.inRange(frame, lower, upper))

        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            r_s, g_s, b_s = map(int, frame[cy, cx])
            depth, x_3d, y_3d, z_3d = self._get_spatial_data(cx, cy)
            detected.append(Landmark(cx, cy, w_box, h_box, r_s, g_s, b_s,
                                     depth=depth, x_3d=x_3d, y_3d=y_3d, z_3d=z_3d))

        detected.sort(key=lambda lm: lm.width * lm.height, reverse=True)
        return detected

    def _get_spatial_data(self, cx, cy):
        """Return spatial coordinates for a pixel location.

        Args:
            cx (int): Pixel x-coordinate.
            cy (int): Pixel y-coordinate.

        Returns:
            tuple: (depth, x_3d, y_3d, z_3d) all None for non-depth cameras.
        """
        return None, None, None, None

    def set_target_colors(self, colors, tolerance=0.05):
        """Set the RGB colors used for landmark detection.

        Args:
            colors (tuple | list[tuple]): One or more (R, G, B) tuples (0-255).
            tolerance (float): Fractional tolerance in [0, 1]. Defaults to 0.05.
        """
        if isinstance(colors, tuple):
            self._target_colors_rgb = [colors]
        else:
            self._target_colors_rgb = list(colors)
        self._color_tolerance = float(np.clip(tolerance, 0.0, 1.0))

    def clear_target_colors(self):
        """Remove all configured target colors."""
        self._target_colors_rgb = []

    @property
    def fps(self):
        """int: Target capture frame rate."""
        return self._fps

    @fps.setter
    def fps(self, value):
        self._fps = max(1, int(value))

    @property
    def rotate_180(self):
        """bool: Whether frames are rotated 180° on capture."""
        return self._rotate_180

    @rotate_180.setter
    def rotate_180(self, flag):
        self._rotate_180 = bool(flag)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop_camera()

    def __del__(self):
        try:
            self.stop_camera()
        except Exception:
            pass