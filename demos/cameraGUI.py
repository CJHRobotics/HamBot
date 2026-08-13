"""HSV color picker / tuner for HamBot.

Click a pixel in the preview to sample its HSV into the sliders. Adjust
the thresholds to see which pixels pass. The numbers here plug straight
into `robot_systems.camera.Camera.set_target_colors`:

    set_target_colors(
        (H, S, V),
        hue_tolerance=...,
        min_saturation=..., min_value=...,   # chromatic
        value_tolerance=...,                 # achromatic
    )

The picker auto-switches between [chromatic] and [achromatic] modes based
on the sampled S: hue-based matching for real colors, value-based
matching for black/white/gray (where hue is meaningless).
OpenCV HSV ranges: H 0-179, S 0-255, V 0-255.
"""

from picamera2 import Picamera2
import numpy as np
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk, ImageDraw  # apt: sudo apt install -y python3-pil.imagetk python3-tk
import cv2

# Kept in sync with robot_systems.camera.Camera.ACHROMATIC_S_THRESHOLD
ACHROMATIC_S_THRESHOLD = 40

# --- Theme -----------------------------------------------------------------
COLORS = {
    "bg":         "#1a1a1a",
    "panel":      "#242424",
    "section":    "#2b2b2b",
    "border":     "#3a3a3a",
    "text":       "#e6e6e6",
    "text_dim":   "#8a8a8a",
    "text_muted": "#6a6a6a",
    "accent":     "#4a9eff",   # chromatic pill
    "achromatic": "#b8b8b8",   # achromatic pill
    "trough":     "#333333",
    "handle":     "#e0e0e0",
    "handle_hi":  "#4a9eff",
    "danger":     "#e05a5a",
}

FONT_UI       = ("DejaVu Sans", 10)
FONT_UI_BOLD  = ("DejaVu Sans", 10, "bold")
FONT_SECTION  = ("DejaVu Sans", 9, "bold")
FONT_MONO     = ("DejaVu Sans Mono", 10)
FONT_MODE     = ("DejaVu Sans", 11, "bold")
FONT_HEADER   = ("DejaVu Sans", 14, "bold")


class HSVPicker:
    def __init__(self, size=(320, 240)):
        self.width, self.height = size

        # --- Camera ---------------------------------------------------------
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)}
        )
        self.picam2.configure(config)
        self.picam2.start()

        # --- Root window ----------------------------------------------------
        self.root = tk.Tk()
        self.root.title("HamBot · HSV Tuner")
        self.root.configure(bg=COLORS["bg"])
        self.root.minsize(720, 520)

        self._apply_ttk_theme()

        outer = tk.Frame(self.root, bg=COLORS["bg"], padx=16, pady=14)
        outer.pack(fill="both", expand=True)

        # --- Left column: controls -----------------------------------------
        left = tk.Frame(outer, bg=COLORS["bg"])
        left.pack(side="left", fill="y", padx=(0, 14))

        self._header(left, "HamBot HSV Tuner",
                     "Click the preview to sample a pixel")

        # Target color section
        target_sec = self._section(left, "TARGET COLOR")
        self.h_slider = self._slider_row(target_sec, "Hue",        0, 179, 0, "H")
        self.s_slider = self._slider_row(target_sec, "Saturation", 0, 255, 0, "S")
        self.v_slider = self._slider_row(target_sec, "Value",      0, 255, 0, "V")

        # Swatch row: shows the sampled color as an RGB square
        swatch_row = tk.Frame(target_sec, bg=COLORS["section"])
        swatch_row.pack(fill="x", pady=(6, 0))
        tk.Label(swatch_row, text="Sampled color",
                 font=FONT_UI, bg=COLORS["section"], fg=COLORS["text_dim"]
                 ).pack(side="left")
        self.swatch = tk.Canvas(swatch_row, width=36, height=20,
                                bg=COLORS["section"], highlightthickness=1,
                                highlightbackground=COLORS["border"])
        self.swatch.pack(side="right")

        # Chromatic thresholds
        chrom_sec = self._section(left, "CHROMATIC THRESHOLDS")
        self.hue_tol_slider = self._slider_row(chrom_sec, "Hue tolerance",   0,  90, 10, "±")
        self.min_s_slider   = self._slider_row(chrom_sec, "Min saturation",  0, 255, 80, "≥")
        self.min_v_slider   = self._slider_row(chrom_sec, "Min value",       0, 255, 60, "≥")

        # Achromatic thresholds
        achr_sec = self._section(left, "ACHROMATIC THRESHOLDS")
        self.value_tol_slider = self._slider_row(achr_sec, "Value tolerance", 0, 128, 30, "±")

        # Contour filter
        cont_sec = self._section(left, "DETECTION")
        self.min_area_slider = self._slider_row(cont_sec, "Min box area", 0, 2000, 100, "px²")

        # Quit button
        quit_btn = tk.Button(left, text="Quit", command=self.quit,
                             font=FONT_UI_BOLD, bg=COLORS["danger"],
                             fg="#ffffff", activebackground="#c04a4a",
                             activeforeground="#ffffff",
                             relief="flat", bd=0, padx=14, pady=8, cursor="hand2")
        quit_btn.pack(fill="x", pady=(14, 0))

        # --- Right column: preview & readout --------------------------------
        right = tk.Frame(outer, bg=COLORS["bg"])
        right.pack(side="left", fill="both", expand=True)

        # Mode pill
        self.mode_pill = tk.Label(right, text="  CHROMATIC  ",
                                  font=FONT_MODE, bg=COLORS["accent"],
                                  fg="#0d0d0d", padx=10, pady=4)
        self.mode_pill.pack(anchor="w", pady=(2, 8))

        # Preview canvas in a bordered panel
        preview_wrap = tk.Frame(right, bg=COLORS["border"], padx=1, pady=1)
        preview_wrap.pack(anchor="w")
        self.canvas = tk.Canvas(preview_wrap, width=self.width, height=self.height,
                                bg="#000000", highlightthickness=0, cursor="crosshair")
        self.canvas.pack()

        # Status readout (monospace)
        self.status = tk.Label(right, text="", font=FONT_MONO,
                               bg=COLORS["bg"], fg=COLORS["text"],
                               justify="left", anchor="w")
        self.status.pack(fill="x", pady=(10, 0))

        tk.Label(right,
                 text="Click any pixel to sample HSV · sliders update live",
                 font=FONT_UI, bg=COLORS["bg"], fg=COLORS["text_muted"]
                 ).pack(anchor="w", pady=(6, 0))

        # State
        self.canvas_img_id = None
        self.photo = None
        self.click_x = self.click_y = None
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.update_image()
        self.root.mainloop()

    # -----------------------------------------------------------------------
    # UI helpers
    # -----------------------------------------------------------------------
    def _apply_ttk_theme(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Dark.Horizontal.TScale",
                        background=COLORS["section"],
                        troughcolor=COLORS["trough"],
                        bordercolor=COLORS["section"],
                        lightcolor=COLORS["handle"],
                        darkcolor=COLORS["handle"])

    def _header(self, parent, title, subtitle):
        tk.Label(parent, text=title, font=FONT_HEADER,
                 bg=COLORS["bg"], fg=COLORS["text"]
                 ).pack(anchor="w")
        tk.Label(parent, text=subtitle, font=FONT_UI,
                 bg=COLORS["bg"], fg=COLORS["text_muted"]
                 ).pack(anchor="w", pady=(0, 10))

    def _section(self, parent, title):
        """Create a titled section box and return the inner frame to pack rows into."""
        tk.Label(parent, text=title, font=FONT_SECTION,
                 bg=COLORS["bg"], fg=COLORS["text_dim"]
                 ).pack(anchor="w", pady=(6, 4))
        box = tk.Frame(parent, bg=COLORS["section"],
                       highlightbackground=COLORS["border"],
                       highlightthickness=1, padx=10, pady=8)
        box.pack(fill="x")
        return box

    def _slider_row(self, parent, label, lo, hi, default, unit=""):
        """One aligned row: [ label · slider · numeric readout ]."""
        row = tk.Frame(parent, bg=COLORS["section"])
        row.pack(fill="x", pady=2)
        row.columnconfigure(1, weight=1)

        tk.Label(row, text=label, font=FONT_UI,
                 bg=COLORS["section"], fg=COLORS["text"],
                 width=14, anchor="w"
                 ).grid(row=0, column=0, sticky="w")

        var = tk.IntVar(value=default)
        readout = tk.Label(row, font=FONT_MONO,
                           bg=COLORS["section"], fg=COLORS["accent"],
                           width=8, anchor="e")
        readout.grid(row=0, column=2, sticky="e", padx=(6, 0))

        def _on_change(v):
            readout.config(text=f"{int(float(v))} {unit}".rstrip())

        scale = ttk.Scale(row, from_=lo, to=hi, orient="horizontal",
                          variable=var, style="Dark.Horizontal.TScale",
                          command=_on_change)
        scale.grid(row=0, column=1, sticky="ew", padx=(8, 0))
        _on_change(default)

        # Attach var to the scale so .get() returns the int
        scale._var = var
        scale.get = lambda: var.get()
        scale.set = lambda v: (var.set(int(v)), _on_change(int(v)))
        return scale

    # -----------------------------------------------------------------------
    # Masking
    # -----------------------------------------------------------------------
    def _mask_achromatic(self, hsv, v_t, v_tol):
        v_lo = max(0, v_t - v_tol)
        v_hi = min(255, v_t + v_tol)
        return cv2.inRange(hsv,
                           np.array([0,   0, v_lo], dtype=np.uint8),
                           np.array([179, ACHROMATIC_S_THRESHOLD, v_hi], dtype=np.uint8))

    def _mask_hsv(self, hsv, h_t, tol, s_min, v_min):
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

    # -----------------------------------------------------------------------
    # Frame loop
    # -----------------------------------------------------------------------
    def _hsv_to_hex(self, h, s, v):
        px = np.uint8([[[h, s, v]]])
        r, g, b = cv2.cvtColor(px, cv2.COLOR_HSV2RGB)[0, 0]
        return f"#{int(r):02x}{int(g):02x}{int(b):02x}"

    def update_image(self):
        arr = self.picam2.capture_array()
        rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

        # Click-to-pick
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

        # Swatch
        self.swatch.configure(bg=self._hsv_to_hex(h_t, s_t, v_t))

        achromatic = s_t < ACHROMATIC_S_THRESHOLD
        if achromatic:
            mask = self._mask_achromatic(hsv, v_t, v_tol)
            self.mode_pill.config(text="  ACHROMATIC  ",
                                  bg=COLORS["achromatic"], fg="#0d0d0d")
        else:
            mask = self._mask_hsv(hsv, h_t, tol, s_min, v_min)
            self.mode_pill.config(text="  CHROMATIC  ",
                                  bg=COLORS["accent"], fg="#0d0d0d")

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        img = Image.fromarray(rgb, mode="RGB")
        draw = ImageDraw.Draw(img)
        boxes = 0
        for c in contours:
            if cv2.contourArea(c) > min_area:
                x, y, w, h = cv2.boundingRect(c)
                draw.rectangle([x, y, x + w, y + h], outline=(80, 255, 120), width=2)
                boxes += 1

        # Status readout
        if achromatic:
            status_text = (f"target  V={v_t:3d}                    ± {v_tol:3d}\n"
                           f"cap     S ≤ {ACHROMATIC_S_THRESHOLD}\n"
                           f"matches {boxes}   min_area {min_area} px²")
        else:
            status_text = (f"target  H={h_t:3d}  S={s_t:3d}  V={v_t:3d}   ± {tol:3d}\n"
                           f"floors  S ≥ {s_min:3d}  V ≥ {v_min:3d}\n"
                           f"matches {boxes}   min_area {min_area} px²")
        self.status.config(text=status_text)

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
