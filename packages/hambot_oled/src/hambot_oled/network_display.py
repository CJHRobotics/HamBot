import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import socket
import subprocess

# OLED setup
oled = adafruit_ssd1306.SSD1306_I2C(128, 32, board.I2C(), addr=0x3C, reset=digitalio.DigitalInOut(board.D4))
font = ImageFont.load_default()

def get_hostname():
    try:
        return socket.gethostname()
    except Exception as e:
        print("Hostname fetch failed:", e)
    return "unknown"

def get_ap_ssid_via_nmcli():
    try:
        output = subprocess.check_output("nmcli -t -f NAME,TYPE connection show --active", shell=True).decode()
        for line in output.strip().split("\n"):
            name, conn_type = line.strip().split(":")
            if conn_type == "802-11-wireless":
                return name
    except Exception as e:
        print("AP mode detection failed:", e)
    return None

def get_display_info():
    # Rule 1: Connected to a Wi-Fi network as client
    try:
        ssid = subprocess.check_output("iwgetid -r", shell=True).decode().strip()
        if ssid:
            return "WiFi", ssid
    except subprocess.CalledProcessError:
        pass

    # Rule 2: Broadcasting an AP
    ap_ssid = get_ap_ssid_via_nmcli()
    if ap_ssid:
        return "AP", ap_ssid

    return None, None

def display_lines(lines):
    oled.fill(0)
    img = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(img)
    for i, text in enumerate(lines[:3]):
        draw.text((0, i * 10), text, font=font, fill=255)
    oled.image(img)
    oled.show()

if __name__ == "__main__":
    mode, ssid = get_display_info()
    if mode and ssid:
        display_lines([
            f"Host: {get_hostname()}",
            f"Mode: {mode}",
            f"SSID: {ssid}"
        ])
    else:
        display_lines([
            f"Host: {get_hostname()}",
            "Mode: --",
            "SSID: No Network"
        ])