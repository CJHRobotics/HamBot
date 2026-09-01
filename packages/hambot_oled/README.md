# HamBot OLED

Shows who a robot is and how to reach it, on a 128x32 SSD1306 OLED via I²C.
Three lines, no labels — at 6px per glyph a prefix like `SSID: ` would eat six
of the ~21 characters a line holds:

```
hambot-07
192.168.50.63
Robobulls_Network_EN..
```

The IP line prefers the address on the Wi-Fi interface, so it describes the
same connection the SSID line names. A robot that hasn't joined a network
falls back to any other address it has — in practice Ethernet — so a Pi on
the bench still shows something you can `ssh` to. In fallback-AP mode the
panel shows `192.168.4.1` and the AP's SSID, which is the robot's own
hostname. Names too long for the panel are truncated with a `..` tail.

## Requirements (system)
- Raspberry Pi OS with I²C enabled (`sudo raspi-config nonint do_i2c 0`)
- `network-manager`, `wireless-tools` (for `nmcli` and `iwgetid`)
- I²C permissions for runtime user (e.g., add user to `i2c` group)

```bash
sudo apt-get update
sudo apt-get install -y network-manager wireless-tools i2c-tools
sudo usermod -aG i2c $USER
```
