import time
import threading
import numpy as np
import cv2
from picamera2 import Picamera2
from robot_systems.camera import Camera


class PiCamera(Camera):
    """Camera backend for the Raspberry Pi Camera Module using picamera2.

    Captures frames in a background thread at the configured FPS and exposes
    them as TRUE RGB numpy arrays.
    """

    def __init__(self, resolution=(640, 480), fps=5, rotate_180=True, tolerance=0.05):
        """Initialise the Pi Camera and start the capture thread.

        Args:
            resolution (tuple[int, int]): (width, height) capture size.
                                          Defaults to (640, 480).
            fps (int): Target capture frame rate. Defaults to 5.
            rotate_180 (bool): Rotate 180° to correct for upside-down mounting.
                               Defaults to True.
            tolerance (float): Fractional color tolerance for landmark
                               detection. Defaults to 0.05.
        """
        super().__init__(fps=fps, rotate_180=rotate_180, tolerance=tolerance)

        self._picam2 = Picamera2()
        self._picam2.configure(
            self._picam2.create_preview_configuration(
                main={"size": resolution, "format": "RGB888"}
            )
        )
        self._picam2.start()

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _capture_loop(self):
        """Continuously capture frames from the Pi Camera at the target FPS."""
        period = 1.0 / float(self._fps)
        next_t = time.monotonic()

        while self._running:
            arr = self._picam2.capture_array()
            if arr is None:
                time.sleep(period)
                continue

            # Guarantee TRUE RGB; some stacks yield BGR even for 'RGB888'
            rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

            if self._rotate_180:
                rgb = rgb[::-1, ::-1]

            with self._frame_lock:
                self._frame_rgb = rgb

            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def get_frame(self, copy=True):
        """Return the latest RGB frame captured by the Pi Camera.

        Args:
            copy (bool): If True return a copy; otherwise the internal array.
                         Defaults to True.

        Returns:
            numpy.ndarray | None: (H, W, 3) uint8 RGB array, or None if no
                                  frame has been captured yet.
        """
        with self._frame_lock:
            if self._frame_rgb is None:
                return None
            return self._frame_rgb.copy() if copy else self._frame_rgb

    def stop_camera(self):
        """Stop the capture thread and release the Pi Camera."""
        if not self._running:
            return
        self._running = False
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        try:
            self._picam2.stop()
        finally:
            self._picam2.close()