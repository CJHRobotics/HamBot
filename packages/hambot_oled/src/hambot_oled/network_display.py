"""Paint the robot's identity onto the 128x32 SSD1306 OLED.

Three lines, no labels — at 6px per glyph a label like "SSID: " costs six of
the ~21 characters a line holds, and the reader already knows what they are
looking at:

    hambot-07
    192.168.50.63
    Robobulls_Network_..

Run once per network state change (NetworkManager dispatcher hook) and once at
boot (hambot_oled.service). See deploy/setup_oled.sh.
"""

import socket
import subprocess

import adafruit_ssd1306
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont

LINE_HEIGHT = 10
ELLIPSIS = ".."
NO_NETWORK = "no network"
NO_IP = "no address"

oled = adafruit_ssd1306.SSD1306_I2C(
    128, 32, board.I2C(), addr=0x3C, reset=digitalio.DigitalInOut(board.D4)
)
font = ImageFont.load_default()


def run(cmd):
    """Stripped stdout of a shell command, or "" if it fails at all.

    Every caller here is best-effort: a missing tool or a down interface
    should blank one line, never crash the display.
    """
    try:
        out = subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL)
        return out.decode(errors="replace").strip()
    except Exception as e:
        print(f"command failed ({cmd}):", e)
        return ""


def get_hostname():
    try:
        return socket.gethostname()
    except Exception as e:
        print("Hostname fetch failed:", e)
        return "unknown"


def get_wireless_device():
    """Name of the first Wi-Fi device (wlan0 in practice), or ""."""
    for line in run("nmcli -t -f DEVICE,TYPE device status").splitlines():
        # DEVICE may contain a colon; TYPE is the last field.
        device, _, dev_type = line.rpartition(":")
        if dev_type == "wifi":
            return device
    return ""


def get_client_ssid():
    """SSID this robot has joined as a client, or "" when it hasn't."""
    return run("iwgetid -r")


def get_ap_ssid(device):
    """SSID this robot is broadcasting — the fallback AP, named after the host."""
    if not device:
        return ""
    # -g escapes nothing but also emits nothing else, so the value is the line.
    return run(f"nmcli -g GENERAL.CONNECTION device show {device}")


def get_device_ip(device):
    """First IPv4 address on `device`, without the /prefix. "" if none."""
    if not device:
        return ""
    for line in run(f"nmcli -g IP4.ADDRESS device show {device}").splitlines():
        address = line.split("/")[0].strip()
        if address:
            return address
    return ""


def get_any_ip():
    """First non-loopback IPv4 anywhere on the box — in practice Ethernet."""
    for address in run("hostname -I").split():
        if ":" not in address and not address.startswith("127."):
            return address
    return ""


def get_network_info():
    """(ip, ssid) for lines two and three.

    The SSID line is wireless-only: the network the robot joined, or the
    fallback AP it is broadcasting when it couldn't join one. The IP line
    prefers the address on that same wireless interface so the two lines
    describe a single connection — and falls back to any other address so a
    robot plugged into Ethernet still shows something you can ssh to.
    """
    device = get_wireless_device()
    ssid = get_client_ssid() or get_ap_ssid(device)
    ip = get_device_ip(device) or get_any_ip()
    return ip or NO_IP, ssid or NO_NETWORK


def fit(draw, text, width):
    """Truncate `text` with a ".." tail until it fits in `width` pixels.

    SSIDs are the reason this exists: "Robobulls_Network_ENG122B" is 25
    characters and the panel holds about 21.
    """
    if draw.textlength(text, font=font) <= width:
        return text
    while text and draw.textlength(text + ELLIPSIS, font=font) > width:
        text = text[:-1]
    return text + ELLIPSIS


def display_lines(lines):
    img = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(img)
    for i, text in enumerate(lines[:3]):
        draw.text((0, i * LINE_HEIGHT), fit(draw, text, oled.width), font=font, fill=255)
    oled.fill(0)
    oled.image(img)
    oled.show()


if __name__ == "__main__":
    ip, ssid = get_network_info()
    display_lines([get_hostname(), ip, ssid])
