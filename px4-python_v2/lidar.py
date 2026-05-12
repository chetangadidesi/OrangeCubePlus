from rplidar import RPLidar
import time
import math
import matplotlib.pyplot as plt

class LidarClass:
    def __init__(self, port='COM5', baudrate=115200, reference_distance=20.0):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.reference_distance = reference_distance  

    def connect(self):
        self.lidar = RPLidar(port=self.port, baudrate=self.baudrate, timeout=3)
        try:
            self.lidar._serial_port.reset_input_buffer()
        except AttributeError:
            pass
        info = self.lidar.get_info()
        print("Lidar info:", info)

    def disconnect(self):
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("Lidar disconnected")

    def run(self):
        plt.ion()
        fig, ax = plt.subplots()
        scatter = ax.scatter([], [], s=2, c='blue')

        ax.set_xlim(-500, 500)
        ax.set_ylim(-500, 500)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title('RPLidar Point Cloud')
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.plot(0, 0, 'ro', markersize=8, label='Lidar')

        print(f"Reference distance: {self.reference_distance:.2f} cm")
        print("=" * 45)

        try:
            self.connect()
            for scan in self.lidar.iter_scans():
                if not plt.fignum_exists(fig.number):
                    print("Window closed, stopping lidar...")
                    break

                points = []
                scan_closest = None  # (distance_cm, angle)

                for quality, angle, distance in scan:
                    if quality > 0 and distance > 0:
                        distance_cm = distance / 10
                        x = distance_cm * math.cos(math.radians(angle))
                        y = distance_cm * math.sin(math.radians(angle))
                        points.append((x, y))

                        if scan_closest is None or distance_cm < scan_closest[0]:
                            scan_closest = (distance_cm, angle)

                # Compute and print error for this scan
                if scan_closest is not None:
                    wall_distance, wall_angle = scan_closest
                    error = wall_distance - self.reference_distance
                    print(f"Wall: {wall_distance:.2f} cm @ {wall_angle:.1f}° | "
                          f"Ref: {self.reference_distance:.2f} cm | "
                          f"Error: {error:+.2f} cm")

                if points:
                    xs, ys = zip(*points)
                    scatter.set_offsets(list(zip(xs, ys)))
                    fig.canvas.draw()
                    fig.canvas.flush_events()

        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.disconnect()
            plt.ioff()

if __name__ == "__main__":
    lidar = LidarClass(port='COM5', reference_distance=20.0)
    lidar.run()
