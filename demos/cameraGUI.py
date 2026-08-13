"""HSV color picker / tuner for HamBot.

Click a pixel in the preview to sample its HSV. Adjust the sliders to see
which pixels get accepted by the current thresholds. The values shown
here plug straight into `robot_systems.camera.Camera.set_target_colors`:

    set_target_colors((H, S, V), hue_tolerance=..., min_saturation=..., min_value=...)

Only hue tolerance is per-target; S/V floors are global (reject washed-out
and dark pixels). OpenCV HSV ranges: H 0-179, S 0-255, V 0-255.
"""

from picamera2 import Picamera2
import numpy as np
import tkinter as tk
from tkinter import Scale, Button, Frame, Label
from PIL import Image, ImageTk, ImageDraw  # apt: sudo apt install -y python3-pil.imagetk python3-tk
import cv2

# Kept in sync with robot_systems.camera.Camera.ACHROMATIC_S_THRESHOLD
ACHROMATIC_S_THRESHOLD = 40


class HSVPicker:
    def __init__(self, size=(320, 240)):
        self.width, self.height = size

        # Camera (request RGB888; some stacks still deliver BGR)
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)}
        )
        self.picam2.configure(config)
        self.picam2.start()

        # Tk layout
        self.root = tk.Tk()
        self.root.title("HSV Color Selector")

        container = Frame(self.root)
        container.pack(fill="both", expand=True)

        left = Frame(container, padx=8, pady=8)
        left.pack(side="left", fill="y")

        Label(left, text="Target HSV (OpenCV ranges)").pack(anchor="w")

        self.h_slider = Scale(left, from_=0, to=179, orient=tk.HORIZONTAL, label="Hue (0-179)")
        self.h_slider.pack(fill="x")
        self.s_slider = Scale(left, from_=0, to=255, orient=tk.HORIZONTAL, label="Saturation (0-255)")
        self.s_slider.pack(fill="x")
        self.v_slider = Scale(left, from_=0, to=255, orient=tk.HORIZONTAL, label="Value (0-255)")
        self.v_slider.pack(fill="x")

        Label(left, text="Match thresholds").pack(anchor="w", pady=(8, 0))

        self.hue_tol_slider = Scale(left, from_=0, to=90, orient=tk.HORIZONTAL, label="Hue tolerance (±)")
        self.hue_tol_slider.set(10)
        self.hue_tol_slider.pack(fill="x")

        self.min_s_slider = Scale(left, from_=0, to=255, orient=tk.HORIZONTAL, label="Min saturation")
        self.min_s_slider.set(80)
        self.min_s_slider.pack(fill="x")

        self.min_v_slider = Scale(left, from_=0, to=255, orient=tk.HORIZONTAL, label="Min value")
        self.min_v_slider.set(60)
        self.min_v_slider.pack(fill="x")

        self.value_tol_slider = Scale(left, from_=0, to=128, orient=tk.HORIZONTAL, label="Value tol (achromatic ±)")
        self.value_tol_slider.set(30)
        self.value_tol_slider.pack(fill="x")

        self.min_area_slider = Scale(left, from_=0, to=2000, orient=tk.HORIZONTAL, label="Min box area (px²)")
        self.min_area_slider.set(100)
        self.min_area_slider.pack(fill="x", pady=(8, 0))

        self.quit_button = Button(left, text="Quit", command=self.quit)
        self.quit_button.pack(pady=(12, 0), fill="x")

        # Preview panel
        right = Frame(container, padx=8, pady=8)
        right.pack(side="right", fill="both", expand=True)

        self.canvas = tk.Canvas(right, width=self.width, height=self.height, highlightthickness=0)
        self.canvas.pack(anchor="n", pady=(0, 4))
        self.info_label = Label(right, text="Click the image to sample HSV")
        self.info_label.pack(anchor="w")

        self.canvas_img_id = None
        self.photo = None  # keep reference
        self.click_x, self.click_y = None, None
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.update_image()
        self.root.mainloop()

    def _mask_achromatic(self, hsv, v_t, v_tol):
        """Build an inRange mask around v_t ± v_tol with S capped at the threshold."""
        v_lo = max(0, v_t - v_tol)
        v_hi = min(255, v_t + v_tol)
        return cv2.inRange(hsv,
                           np.array([0,   0, v_lo], dtype=np.uint8),
                           np.array([179, ACHROMATIC_S_THRESHOLD, v_hi], dtype=np.uint8))

    def _mask_hsv(self, hsv, h_t, tol, s_min, v_min):
        """Build an inRange mask around h_t ± tol with hue wraparound."""
        h_lo, h_hi = h_t - tol, h_t + tol
        if h_lo < 0:
            m1 = cv2.inRange(hsv, np.array([0, s_min, v_min], dtype=np.uint8),
                                  np.array([h_hi, 255, 255], dtype=np.uint8))
            m2 = cv2.inRange(hsv, np.array([180 + h_lo, s_min, v_min], dtype=np.uint8),
                                  np.array([179, 255, 255], dtype=np.uint8))
            return cv2.bitwise_or(m1, m2)
        if h_hi > 179:
            m1 = cv2.inRange(hsv, np.array([h_lo, s_min, v_min], dtype=np.uint8),
                                  np.array([179, 255, 255], dtype=np.uint8))
            m2 = cv2.inRange(hsv, np.array([0, s_min, v_min], dtype=np.uint8),
                                  np.array([h_hi - 180, 255, 255], dtype=np.uint8))
            return cv2.bitwise_or(m1, m2)
        return cv2.inRange(hsv, np.array([h_lo, s_min, v_min], dtype=np.uint8),
                                np.array([h_hi, 255, 255], dtype=np.uint8))

    def update_image(self):
        arr = self.picam2.capture_array()
        # Normalize to true RGB (some stacks return BGR even for 'RGB888')
        rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        # Click-to-pick reads HSV from the sampled pixel
        if self.click_x is not None and self.click_y is not None:
            x = int(np.clip(self.click_x, 0, self.width - 1))
            y = int(np.clip(self.click_y, 0, self.height - 1))
            h_p, s_p, v_p = hsv[y, x]
            self.h_slider.set(int(h_p))
            self.s_slider.set(int(s_p))
            self.v_slider.set(int(v_p))
            self.click_x = self.click_y = None

        h_t = self.h_slider.get()
        s_t = self.s_slider.get()
        v_t = self.v_slider.get()
        tol = self.hue_tol_slider.get()
        s_min = self.min_s_slider.get()
        v_min = self.min_v_slider.get()
        v_tol = self.value_tol_slider.get()
        min_area = self.min_area_slider.get()

        achromatic = s_t < ACHROMATIC_S_THRESHOLD
        if achromatic:
            mask = self._mask_achromatic(hsv, v_t, v_tol)
        else:
            mask = self._mask_hsv(hsv, h_t, tol, s_min, v_min)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img = Image.fromarray(rgb, mode="RGB")
        draw = ImageDraw.Draw(img)
        for c in contours:
            if cv2.contourArea(c) > min_area:
                x, y, w, h = cv2.boundingRect(c)
                draw.rectangle([x, y, x + w, y + h], outline=(0, 255, 0), width=2)

        if achromatic:
            info = (f"[achromatic]  target V={v_t}  ±{v_tol}   "
                    f"(S≤{ACHROMATIC_S_THRESHOLD})   min_area={min_area}")
        else:
            info = (f"[chromatic]   target H={h_t} S={s_t} V={v_t}   "
                    f"±{tol}   floors S≥{s_min} V≥{v_min}   min_area={min_area}")
        self.info_label.config(text=info)

        self.photo = ImageTk.PhotoImage(image=img)
        if self.canvas_img_id is None:
            self.canvas_img_id = self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
        else:
            self.canvas.itemconfig(self.canvas_img_id, image=self.photo)

        self.root.after(30, self.update_image)

    def _on_canvas_click(self, event):
        self.click_x, self.click_y = event.x, event.y

    def quit(self):
        try: self.picam2.stop()
        except Exception: pass
        try: self.root.quit()
        except Exception: pass
        try: self.root.destroy()
        except Exception: pass


def main():
    HSVPicker()


if __name__ == "__main__":
    main()
