import time
import threading
import numpy as np
import cv2
import depthai as dai
from robot_systems.camera import Camera


class OakCamera(Camera):
    """Camera backend for the Luxonis OAK-D Lite using DepthAI.

    Builds a DepthAI pipeline with an RGB camera and stereo depth, captures
    frames in a background thread, and exposes RGB, depth, and disparity data.
    Landmark detection inherits from Camera and additionally populates 3D
    spatial coordinates via the SpatialLocationCalculator node.
    """

    def __init__(self, resolution=(640, 480), fps=5, rotate_180=False, tolerance=0.05):
        """Initialise the OAK-D Lite pipeline and start the capture thread.

        Args:
            resolution (tuple[int, int]): (width, height) for the RGB stream.
                                          Defaults to (640, 480).
            fps (int): Target capture frame rate. Defaults to 5.
            rotate_180 (bool): Rotate 180° to correct for upside-down mounting.
                               Defaults to False.
            tolerance (float): Fractional color tolerance for landmark
                               detection. Defaults to 0.05.
        """
        super().__init__(fps=fps, rotate_180=rotate_180, tolerance=tolerance)

        self._resolution = resolution
        self._depth_frame      = None
        self._disparity_frame  = None
        self._depth_lock       = threading.Lock()

        self._device, self._rgb_queue, self._depth_queue, \
            self._disparity_queue, self._spatial_calc, \
            self._spatial_in, self._spatial_out = self._build_pipeline()

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

    def _build_pipeline(self):
        """Build and start the DepthAI pipeline.

        Returns:
            tuple: (device, rgb_queue, depth_queue, disparity_queue,
                    spatial_calc_node, spatial_in_queue, spatial_out_queue)
        """
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(*self._resolution)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam_rgb.setFps(self._fps)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        mono_left  = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo     = pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetType.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        xout_disp = pipeline.create(dai.node.XLinkOut)
        xout_disp.setStreamName('disparity')
        stereo.disparity.link(xout_disp.input)

        spatial_calc = pipeline.create(dai.node.SpatialLocationCalculator)
        spatial_calc.inputDepth.setBlocking(False)
        spatial_calc.inputDepth.setQueueSize(1)
        stereo.depth.link(spatial_calc.inputDepth)

        xout_spatial = pipeline.create(dai.node.XLinkOut)
        xout_spatial.setStreamName('spatial')
        spatial_calc.out.link(xout_spatial.input)

        xin_spatial = pipeline.create(dai.node.XLinkIn)
        xin_spatial.setStreamName('spatial_cfg')
        xin_spatial.out.link(spatial_calc.inputConfig)

        device = dai.Device(pipeline)
        rgb_queue      = device.getOutputQueue('rgb',      maxSize=4, blocking=False)
        depth_queue    = device.getOutputQueue('depth',    maxSize=4, blocking=False)
        disparity_queue= device.getOutputQueue('disparity',maxSize=4, blocking=False)
        spatial_out    = device.getOutputQueue('spatial',  maxSize=4, blocking=False)
        spatial_in     = device.getInputQueue('spatial_cfg')

        return device, rgb_queue, depth_queue, disparity_queue, \
               spatial_calc, spatial_in, spatial_out

    def _capture_loop(self):
        """Continuously pull frames from the DepthAI output queues."""
        period = 1.0 / float(self._fps)
        next_t = time.monotonic()

        while self._running:
            in_rgb = self._rgb_queue.tryGet()
            if in_rgb is not None:
                rgb = in_rgb.getCvFrame()
                if self._rotate_180:
                    rgb = rgb[::-1, ::-1]
                with self._frame_lock:
                    self._frame_rgb = rgb

            in_depth = self._depth_queue.tryGet()
            if in_depth is not None:
                with self._depth_lock:
                    self._depth_frame = in_depth.getFrame()

            in_disp = self._disparity_queue.tryGet()
            if in_disp is not None:
                disp = in_disp.getFrame()
                with self._depth_lock:
                    self._disparity_frame = disp

            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def get_frame(self, copy=True):
        """Return the latest RGB frame from the OAK-D Lite.

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

    def get_depth_frame(self, copy=True):
        """Return the latest depth frame in millimetres.

        Args:
            copy (bool): If True return a copy. Defaults to True.

        Returns:
            numpy.ndarray | None: (H, W) uint16 array of depth values in mm,
                                  or None if no frame is available yet.
        """
        with self._depth_lock:
            if self._depth_frame is None:
                return None
            return self._depth_frame.copy() if copy else self._depth_frame

    def get_disparity_frame(self, copy=True):
        """Return the latest raw disparity frame.

        Args:
            copy (bool): If True return a copy. Defaults to True.

        Returns:
            numpy.ndarray | None: (H, W) disparity array, or None if not available.
        """
        with self._depth_lock:
            if self._disparity_frame is None:
                return None
            return self._disparity_frame.copy() if copy else self._disparity_frame

    def get_depth_at(self, cx, cy, roi_size=10):
        """Return the median depth value in mm at a pixel location.

        Args:
            cx (int): Pixel x-coordinate.
            cy (int): Pixel y-coordinate.
            roi_size (int): Half-size of the sampling region in pixels.
                            Defaults to 10.

        Returns:
            float | None: Median depth in mm, or None if no depth frame is available.
        """
        depth = self.get_depth_frame(copy=False)
        if depth is None:
            return None
        h, w = depth.shape
        x0 = max(0, cx - roi_size)
        x1 = min(w, cx + roi_size)
        y0 = max(0, cy - roi_size)
        y1 = min(h, cy + roi_size)
        region = depth[y0:y1, x0:x1]
        valid = region[region > 0]
        return float(np.median(valid)) if valid.size > 0 else None

    def _get_spatial_data(self, cx, cy):
        """Return 3D spatial coordinates for a pixel location.

        Args:
            cx (int): Pixel x-coordinate.
            cy (int): Pixel y-coordinate.

        Returns:
            tuple: (depth_mm, x_3d_mm, y_3d_mm, z_3d_mm) or all None on failure.
        """
        try:
            frame = self.get_frame(copy=False)
            if frame is None:
                return None, None, None, None

            h, w = frame.shape[:2]
            roi_half = 0.02
            xn = cx / w
            yn = cy / h

            cfg = dai.SpatialLocationCalculatorConfig()
            roi = dai.SpatialLocationCalculatorConfigData()
            roi.roi = dai.Rect(
                dai.Point2f(max(0.0, xn - roi_half), max(0.0, yn - roi_half)),
                dai.Point2f(min(1.0, xn + roi_half), min(1.0, yn + roi_half))
            )
            roi.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
            cfg.addROI(roi)
            self._spatial_in.send(cfg)

            result = self._spatial_out.tryGet()
            if result is None:
                return None, None, None, None

            for data in result.getSpatialLocations():
                coords = data.spatialCoordinates
                depth_mm = float(coords.z)
                return depth_mm, float(coords.x), float(coords.y), float(coords.z)

        except Exception:
            pass

        return None, None, None, None

    def stop_camera(self):
        """Stop the capture thread and close the OAK-D device."""
        if not self._running:
            return
        self._running = False
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        try:
            self._device.close()
        except Exception:
            pass