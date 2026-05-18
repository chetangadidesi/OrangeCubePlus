import threading
from rplidar import RPLidar


class LidarHandler:
    def __init__(self, port='COM5', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.running = False
        self.thread = None
        self._lock = threading.Lock()

        self.current_distance = None  # cm
        self.current_angle = None     # degrees

    def connect(self):
        self.lidar = RPLidar(port=self.port, baudrate=self.baudrate, timeout=3)
        try:
            self.lidar._serial_port.reset_input_buffer()
        except AttributeError:
            pass
        info = self.lidar.get_info()
        print("Lidar info:", info)

    def start(self):
        self.connect()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        print("LidarHandler started.")

    def stop(self):
        self.running = False
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("LidarHandler stopped.")
        if self.thread:
            self.thread.join(timeout=1.0)

    def _update(self):
        """Background thread: continuously finds the closest point each scan."""
        for scan in self.lidar.iter_scans():
            if not self.running:
                break

            closest = None
            for quality, angle, distance in scan:
                if quality > 0 and distance > 0:
                    distance_cm = distance / 10
                    if closest is None or distance_cm < closest[0]:
                        closest = (distance_cm, angle)

            if closest:
                with self._lock:
                    self.current_distance = closest[0]
                    self.current_angle = closest[1]

    def get_distance(self):
        with self._lock:
            return self.current_distance

    def get_angle(self):
        with self._lock:
            return self.current_angle
