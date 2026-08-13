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

    def __init__(self, fps=5, rotate_180=False,
                 hue_tolerance=10, min_saturation=80, min_value=60):
        """Initialise shared camera state.

        Args:
            fps (int): Target capture frame rate. Defaults to 5.
            rotate_180 (bool): Rotate frame 180° to correct for upside-down
                               mounting. Defaults to False.
            hue_tolerance (int): ± hue units around each target for matching.
                                 OpenCV hue is 0-179 (half the 360° wheel), so
                                 a tolerance of 10 covers a 20-wide band.
                                 Defaults to 10.
            min_saturation (int): Minimum S (0-255) for a pixel to be considered.
                                  Rejects washed-out / gray pixels. Defaults to 80.
            min_value (int): Minimum V (0-255) for a pixel to be considered.
                             Rejects shadows and near-black pixels. Defaults to 60.
        """
        self._fps            = max(1, int(fps))
        self._rotate_180     = bool(rotate_180)
        self._frame_rgb      = None
        self._frame_lock     = threading.Lock()
        self._target_hsv     = []
        self._hue_tolerance  = int(np.clip(hue_tolerance, 0, 90))
        self._min_saturation = int(np.clip(min_saturation, 0, 255))
        self._min_value      = int(np.clip(min_value, 0, 255))
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
        """Detect regions matching any configured target color (HSV).

        A pixel matches a target if its hue is within `hue_tolerance` of the
        target hue (with wraparound at 0/179) AND its saturation and value
        clear the global `min_saturation`/`min_value` floors. Hue is the only
        per-target knob; S and V are treated as global quality filters that
        reject washed-out or dark pixels.

        Args:
            min_area (int): Minimum contour area in pixels to report.
                            Defaults to 500.

        Returns:
            list[Landmark]: Detected landmarks sorted by area (largest first).
        """
        frame = self.get_frame(copy=False)
        if frame is None or not self._target_hsv:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        h_img, w_img = frame.shape[:2]
        mask_total = np.zeros((h_img, w_img), dtype=np.uint8)

        s_min = self._min_saturation
        v_min = self._min_value
        tol   = self._hue_tolerance

        for (h_t, _s_t, _v_t) in self._target_hsv:
            h_lo = h_t - tol
            h_hi = h_t + tol
            if h_lo < 0:
                # Wrap: [0, h_hi] ∪ [180 + h_lo, 179]
                lower1 = np.array([0,             s_min, v_min], dtype=np.uint8)
                upper1 = np.array([h_hi,          255,   255],   dtype=np.uint8)
                lower2 = np.array([180 + h_lo,    s_min, v_min], dtype=np.uint8)
                upper2 = np.array([179,           255,   255],   dtype=np.uint8)
                mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1),
                                      cv2.inRange(hsv, lower2, upper2))
            elif h_hi > 179:
                # Wrap: [h_lo, 179] ∪ [0, h_hi - 180]
                lower1 = np.array([h_lo,          s_min, v_min], dtype=np.uint8)
                upper1 = np.array([179,           255,   255],   dtype=np.uint8)
                lower2 = np.array([0,             s_min, v_min], dtype=np.uint8)
                upper2 = np.array([h_hi - 180,    255,   255],   dtype=np.uint8)
                mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1),
                                      cv2.inRange(hsv, lower2, upper2))
            else:
                lower = np.array([h_lo, s_min, v_min], dtype=np.uint8)
                upper = np.array([h_hi, 255,   255],   dtype=np.uint8)
                mask  = cv2.inRange(hsv, lower, upper)
            mask_total = cv2.bitwise_or(mask_total, mask)

        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            h_s, s_s, v_s = map(int, hsv[cy, cx])
            depth, x_3d, y_3d, z_3d = self._get_spatial_data(cx, cy)
            detected.append(Landmark(cx, cy, w_box, h_box, h_s, s_s, v_s,
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

    def set_target_colors(self, colors, hue_tolerance=None,
                          min_saturation=None, min_value=None):
        """Set the HSV colors used for landmark detection.

        Args:
            colors (tuple | list[tuple]): One or more (H, S, V) tuples in
                OpenCV ranges — H: 0-179, S: 0-255, V: 0-255. Only H is used
                for matching; the S/V values in each tuple are informational
                (e.g., what the picker sampled) and can be left as 0.
            hue_tolerance (int | None): ± hue units around each target.
                                        If None, the current value is kept.
            min_saturation (int | None): Global S floor. If None, kept as-is.
            min_value (int | None): Global V floor. If None, kept as-is.
        """
        if isinstance(colors, tuple):
            self._target_hsv = [colors]
        else:
            self._target_hsv = list(colors)
        if hue_tolerance is not None:
            self._hue_tolerance = int(np.clip(hue_tolerance, 0, 90))
        if min_saturation is not None:
            self._min_saturation = int(np.clip(min_saturation, 0, 255))
        if min_value is not None:
            self._min_value = int(np.clip(min_value, 0, 255))

    def clear_target_colors(self):
        """Remove all configured target colors."""
        self._target_hsv = []

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