import time
import threading
from math import floor
from adafruit_rplidar import RPLidar, RPLidarException


class Lidar:
    INVALID_READING = -1  # Define a constant for invalid readings

    # A slot that has never been written reads as infinitely old rather than
    # as "written at time zero", so a fresh start cannot look like fresh data.
    _NEVER = float("-inf")

    def __init__(self, port_name="/dev/ttyUSB0", timeout=3, stale_after_s=1.0):
        """
        Initialize the RPLidar, start the motor, and begin scanning in a separate thread.

        Args:
            port_name (str): The port name where the Lidar is connected.
            timeout (int): The timeout for the Lidar connection.
            stale_after_s (float): Default age, in seconds, past which a slot is
                reported as INVALID_READING instead of as a valid distance. The
                default of 1.0 s reasons from a 5-10 Hz spin, which refreshes
                every visible angle every 100-200 ms; a slot quiet for a full
                second is one nothing has bounced off recently, not a slot
                measured as empty. Raise it if a real unit leaves legitimately
                visible angles quiet for longer.
        """
        self.lidar = RPLidar(None, port_name,timeout = timeout)
        self.lidar.start_motor()
        self.scan_data = [self.INVALID_READING] * 360  # Initialize scan data with 360 degrees
        # Monotonic write time per slot, parallel to scan_data. Monotonic
        # because expiry must not jump when wall-clock time is corrected.
        self.scan_time = [self._NEVER] * 360
        self.stale_after_s = float(stale_after_s)
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
                        slot = min(359, floor(adjusted_angle))
                        # Distance first, timestamp second. A reader that
                        # interleaves between the two then sees a new distance
                        # carrying an old timestamp and discards it as stale,
                        # which is the safe direction to lose this race: a
                        # good reading is dropped, never a stale one served.
                        if distance > 0:  # Only update if the distance is valid
                            self.scan_data[slot] = distance
                        else:
                            self.scan_data[slot] = self.INVALID_READING
                        self.scan_time[slot] = time.monotonic()
                    if not self.running:
                        break
            except RPLidarException as e:
                print(f"RPLidar exception occurred: {e}")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")


    def get_current_scan(self, max_age_s=-1.0):
        """
        Get the current scan data as a list of 360 distance measurements.

        Slots not written within the age window are returned as
        INVALID_READING, so "nothing is there" stays distinguishable from
        "nothing has been measured there recently".

        Args:
            max_age_s (float): Age window in seconds. A negative value (the
                default) uses the instance's ``stale_after_s``. Pass None to
                skip expiry entirely and get the raw array.

        Returns:
            list: A 360-element list of distances in mm, with expired slots
                  replaced by INVALID_READING.
        """
        if max_age_s is None:
            return self.scan_data.copy()

        window = self.stale_after_s if max_age_s < 0 else float(max_age_s)
        now = time.monotonic()
        data = self.scan_data
        times = self.scan_time
        return [
            data[i] if (now - times[i]) <= window else self.INVALID_READING
            for i in range(360)
        ]

    def slot_age(self, degree):
        """
        Seconds since a given slot was last written.

        Args:
            degree (int): Angle in degrees; wrapped into 0-359.

        Returns:
            float: Age in seconds, or inf if the slot has never been written.
        """
        t = self.scan_time[int(degree) % 360]
        if t == self._NEVER:
            return float("inf")
        return max(0.0, time.monotonic() - t)

    def sample(self, max_age_s=-1.0):
        """
        Return a scan and the write time of its freshest slot, together.

        Used by ``HamBot.step()`` to latch the scan with a reading time. A scan
        is a mosaic of per-slot times, so there is no single instant it
        describes; the newest slot's time is the most honest single stamp for
        it. Per-slot ages remain available via ``slot_age()``.

        Args:
            max_age_s (float): Passed through to ``get_current_scan()``.

        Returns:
            tuple: (scan_list, newest_write_time) where newest_write_time is
                   None if no slot has ever been written.
        """
        scan = self.get_current_scan(max_age_s=max_age_s)
        newest = max(self.scan_time)
        return scan, (None if newest == self._NEVER else newest)

    def stop_lidar(self):
        """
        Safely stop the Lidar and the scanning thread, and disconnect the device.
        """
        self.running = False
        self.scan_thread.join()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
