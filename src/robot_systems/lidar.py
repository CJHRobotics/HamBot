import time
import threading
from math import floor
from adafruit_rplidar import RPLidar, RPLidarException


class Lidar:
    INVALID_READING = -1
    MOTOR_SPINUP_DELAY = 2

    def __init__(self, port_name="/dev/ttyUSB0"):
        """Initialize the RPLidar, start the motor, and begin scanning in a background thread.

        Args:
            port_name (str): Serial port the lidar is connected to. Defaults to /dev/ttyUSB0.
        """
        self.lidar = RPLidar(None, port_name)
        self.lidar.start_motor()
        time.sleep(self.MOTOR_SPINUP_DELAY)
        self.scan_data = [self.INVALID_READING] * 360
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan)
        self.scan_thread.start()

    def _scan(self):
        """Continuously read scans from the lidar and update scan_data.

        Runs in a background thread. Blocks on iter_scans() which is naturally
        paced by the hardware rotation rate (~5–10 Hz).
        """
        while self.running:
            try:
                for scan in self.lidar.iter_scans():
                    for _, angle, distance in scan:
                        adjusted_angle = (angle + 180) % 360
                        if distance > 0:
                            self.scan_data[min(359, floor(adjusted_angle))] = distance
                        else:
                            self.scan_data[min(359, floor(adjusted_angle))] = -1
                    if not self.running:
                        break
            except RPLidarException as e:
                print(f"RPLidar exception occurred: {e}")
                time.sleep(1)
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
                time.sleep(1)

    def get_current_scan(self):
        """Return a snapshot of the current 360° scan data.

        Returns:
            list: 360 distance values in mm, indexed by degree.
        """
        return self.scan_data.copy()

    def stop_lidar(self):
        """Stop the scan thread and disconnect the lidar hardware."""
        self.running = False
        self.scan_thread.join()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()