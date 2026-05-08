import time
import sys
class ProportionalController:
    def __init__(self, mav_handler, motor_controller, kp=0.01, base_throttle=0.15):
        self.mav = mav_handler
        self.motors = motor_controller
        self.kp = kp
        self.base_throttle = base_throttle
        self.target_roll = 0.0  # target: level

    def get_roll(self):
        return self.mav.curr_roll  # live roll in degrees

    def compute(self):
        roll = self.get_roll()
        error = self.target_roll - roll
        correction = self.kp * error

        # Quad motor layout (adjust to match your frame):
        # M1 (front-right), M2 (back-left), M3 (front-left), M4 (back-right)
        # Roll right = increase left motors, decrease right motors
        m1 = self.base_throttle - correction  # front-right
        m2 = self.base_throttle + correction  # back-left
        m3 = self.base_throttle + correction  # front-left
        m4 = self.base_throttle - correction  # back-right

        # Clamp all to safe range
        m1, m2, m3, m4 = [max(0.0, min(1.0, m)) for m in [m1, m2, m3, m4]]

        return m1, m2, m3, m4

    def run(self, duration_sec=10, loop_hz=10):
        interval = 1.0 / loop_hz
        end_time = time.time() + duration_sec

        while time.time() < end_time:
            m1, m2, m3, m4 = self.compute()
            roll = self.get_roll()
            correction = self.kp * (self.target_roll - roll)

            sys.stdout.write(
                f"\r  Roll:{roll:>7.2f} deg | Correction:{correction:>7.3f} | M1:{m1:.3f} M2:{m2:.3f} M3:{m3:.3f} M4:{m4:.3f}"
            )
            sys.stdout.flush()

            self.motors.actuator_test(1, m1, interval * 2)
            self.motors.actuator_test(2, m2, interval * 2)
            self.motors.actuator_test(3, m3, interval * 2)
            self.motors.actuator_test(4, m4, interval * 2)

            time.sleep(interval)

        for m in [1, 2, 3, 4]:
            self.motors.actuator_test(m, 0.0, 1)
        print("Controller stopped.")