import time
import threading
from math import floor
from adafruit_rplidar import RPLidar, RPLidarException


class Lidar:
    INVALID_READING = -1  # Define a constant for invalid readings

    def __init__(self, port_name="/dev/ttyUSB0"):
        """
        Initialize the RPLidar, start the motor, and begin scanning in a separate thread.

        Args:
            port_name (str): The port name where the Lidar is connected.
            timeout (int): The timeout for the Lidar connection.
        """
        self.lidar = RPLidar(None, port_name)
        self.lidar.start_motor()
        self.scan_data = [self.INVALID_READING] * 360  # Initialize scan data with 360 degrees
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan)
        self.scan_thread.start()


    def _scan(self):
        """
        Continuously scan and update the range data in a 360-degree list.
        This method runs in a separate thread.
        """
        while self.running:
            try:
                for scan in self.lidar.iter_scans():
                    for _, angle, distance in scan:
                        # Convert angle to match the required orientation (0° at back, 180° at front)
                        adjusted_angle = (angle + 180) % 360
                        if distance > 0:  # Only update if the distance is valid
                            self.scan_data[min(359, floor(adjusted_angle))] = distance
                        else:
                            self.scan_data[min(359, floor(adjusted_angle))] = -1
                    if not self.running:
                        break
            except RPLidarException as e:
                print(f"RPLidar exception occurred: {e}")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")


    def get_current_scan(self):
        """
        Get the current scan data as a list of 360 distance measurements.

        Returns:
            list: A copy of the current scan data.
        """
        return self.scan_data.copy()

    def stop_lidar(self):
        """
        Safely stop the Lidar and the scanning thread, and disconnect the device.
        """
        self.running = False
        self.scan_thread.join()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
