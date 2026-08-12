from picamera2 import Picamera2
import numpy as np
import tkinter as tk
from tkinter import Scale, Button, Frame, Label
from PIL import Image, ImageTk, ImageDraw  # apt: sudo apt install -y python3-pil.imagetk python3-tk
import cv2  # used for inRange, findContours, and BGR->RGB conversion

class SimpleRGBPicker:
    def __init__(self, size=(320, 240)):
        self.width, self.height = size

        # Camera (request RGB888; some stacks still deliver BGR)
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)}
        )
        self.picam2.configure(config)
        self.picam2.start()

        # Tk root and two-panel layout
        self.root = tk.Tk()
        self.root.title("RGB Color Selector")

        container = Frame(self.root)
        container.pack(fill="both", expand=True)

        # Left controls
        left = Frame(container, padx=8, pady=8)
        left.pack(side="left", fill="y")

        Label(left, text="Target RGB").pack(anchor="w")

        self.red_slider = Scale(left, from_=0, to=255, orient=tk.HORIZONTAL, label="Red")
        self.red_slider.pack(fill="x")
        self.green_slider = Scale(left, from_=0, to=255, orient=tk.HORIZONTAL, label="Green")
        self.green_slider.pack(fill="x")
        self.blue_slider = Scale(left, from_=0, to=255, orient=tk.HORIZONTAL, label="Blue")
        self.blue_slider.pack(fill="x")

        self.tolerance_slider = Scale(left, from_=0, to=100, orient=tk.HORIZONTAL, label="Tolerance (%)")
        self.tolerance_slider.set(10)
        self.tolerance_slider.pack(fill="x", pady=(8, 0))

        self.quit_button = Button(left, text="Quit", command=self.quit)
        self.quit_button.pack(pady=(12, 0), fill="x")

        # Right preview
        right = Frame(container, padx=8, pady=8)
        right.pack(side="right", fill="both", expand=True)

        self.canvas = tk.Canvas(right, width=self.width, height=self.height, highlightthickness=0)
        self.canvas.pack(anchor="n", pady=(0, 4))
        Label(right, text="Click the image to pick color").pack(anchor="w")

        self.canvas_img_id = None
        self.photo = None  # keep reference
        self.click_x, self.click_y = None, None
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.update_image()
        self.root.mainloop()

    def update_image(self):
        # Capture and flip vertically (orientation only)
        arr = self.picam2.capture_array()
        arr = np.flipud(arr)

        # Normalize to TRUE RGB for the GUI and all logic
        # (Some pipelines hand back BGR even when 'RGB888' is requested)
        rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

        # Sliders
        r = self.red_slider.get()
        g = self.green_slider.get()
        b = self.blue_slider.get()
        tol = self.tolerance_slider.get() / 100.0

        lower = np.array([max(0, r - int(255 * tol)),
                          max(0, g - int(255 * tol)),
                          max(0, b - int(255 * tol))], dtype=np.uint8)
        upper = np.array([min(255, r + int(255 * tol)),
                          min(255, g + int(255 * tol)),
                          min(255, b + int(255 * tol))], dtype=np.uint8)

        # Mask/contours in RGB space
        mask = cv2.inRange(rgb, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Click-to-pick from displayed RGB
        if self.click_x is not None and self.click_y is not None:
            x = int(np.clip(self.click_x, 0, self.width - 1))
            y = int(np.clip(self.click_y, 0, self.height - 1))
            picked = rgb[y, x]  # [R, G, B]
            self.red_slider.set(int(picked[0]))
            self.green_slider.set(int(picked[1]))
            self.blue_slider.set(int(picked[2]))
            self.click_x = self.click_y = None

        # Draw boxes in RGB using Pillow (keep colors accurate)
        img = Image.fromarray(rgb, mode="RGB")
        draw = ImageDraw.Draw(img)
        for c in contours:
            if cv2.contourArea(c) > 100:
                x, y, w, h = cv2.boundingRect(c)
                draw.rectangle([x, y, x + w, y + h], outline=(0, 255, 0), width=2)

        # Show on Tk canvas
        self.photo = ImageTk.PhotoImage(image=img)
        if self.canvas_img_id is None:
            self.canvas_img_id = self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
        else:
            self.canvas.itemconfig(self.canvas_img_id, image=self.photo)

        self.root.after(30, self.update_image)

    def _on_canvas_click(self, event):
        self.click_x, self.click_y = event.x, event.y

    def quit(self):
        try:
            self.picam2.stop()
        except Exception:
            pass
        try:
            self.root.quit()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

def main():
    SimpleRGBPicker()

if __name__ == "__main__":
    main()
