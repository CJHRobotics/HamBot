# HamBot OLED

Displays current network mode/SSID/IP on a 128x32 SSD1306 OLED via I²C.

## Requirements (system)
- Raspberry Pi OS with I²C enabled (`sudo raspi-config nonint do_i2c 0`)
- `network-manager`, `wireless-tools` (for `nmcli` and `iwgetid`)
- I²C permissions for runtime user (e.g., add user to `i2c` group)

```bash
sudo apt-get update
sudo apt-get install -y network-manager wireless-tools i2c-tools
sudo usermod -aG i2c $USER
