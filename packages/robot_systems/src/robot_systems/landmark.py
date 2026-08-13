class Landmark:
    def __init__(self, x, y, width, height, h, s, v,
                 depth=None, x_3d=None, y_3d=None, z_3d=None):
        """Represents a detected landmark in the camera frame.

        Args:
            x (int): X-coordinate of the bounding box center in pixels.
            y (int): Y-coordinate of the bounding box center in pixels.
            width (int): Width of the bounding box in pixels.
            height (int): Height of the bounding box in pixels.
            h (int): Hue component of the sampled center pixel (OpenCV, 0-179).
            s (int): Saturation of the sampled center pixel (0-255).
            v (int): Value/brightness of the sampled center pixel (0-255).
            depth (float | None): Distance to the landmark in mm. Populated by
                                  OAK-D only; None for Pi Camera.
            x_3d (float | None): X spatial coordinate in mm (OAK-D only).
            y_3d (float | None): Y spatial coordinate in mm (OAK-D only).
            z_3d (float | None): Z spatial coordinate in mm (OAK-D only).
        """
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.h = h
        self.s = s
        self.v = v
        self.depth = depth
        self.x_3d = x_3d
        self.y_3d = y_3d
        self.z_3d = z_3d

    def has_spatial_data(self):
        """Return True if 3D spatial coordinates are available."""
        return self.z_3d is not None

    def __repr__(self):
        base = (f"Landmark(x={self.x}, y={self.y}, w={self.width}, h={self.height}, "
                f"H={self.h}, S={self.s}, V={self.v}")
        if self.has_spatial_data():
            base += f", depth={self.depth:.0f}mm, 3D=({self.x_3d:.0f},{self.y_3d:.0f},{self.z_3d:.0f})mm"
        return base + ")"
