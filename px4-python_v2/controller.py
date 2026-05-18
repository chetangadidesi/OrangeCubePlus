import time
import sys
import math


class ProportionalController:
    def __init__(self, mav_handler, motor_controller, lidar_handler,
                 kp_roll=0.01,
                 kp_pitch_inner=0.01,
                 kp_pitch_outer=0.5,
                 base_throttle=0.15,
                 reference_distance=20.0,
                 max_pitch=10.0,
                 roll_enabled=True,
                 pitch_enabled=True):
        
        self.mav    = mav_handler
        self.motors = motor_controller
        self.lidar  = lidar_handler

        # Gains
        self.kp_roll         = kp_roll           # roll inner loop
        self.kp_pitch_inner  = kp_pitch_inner     # pitch inner loop
        self.kp_pitch_outer  = kp_pitch_outer     # distance → target pitch

        self.base_throttle      = base_throttle
        self.reference_distance = reference_distance  # cm
        self.max_pitch          = max_pitch           # degrees, safety clamp

        self.target_roll = 0.0  # always stabilize roll to level
        self.roll_enabled = roll_enabled
        self.pitch_enabled = pitch_enabled

    def get_roll(self):
        return self.mav.curr_roll

    def get_pitch(self):
        return self.mav.curr_pitch

    def compute(self):
        # -------------------------------------------------------
        # ROLL: stabilize to 0°  
        # -------------------------------------------------------
        roll = self.get_roll()
        roll_correction = self.kp_roll * (self.target_roll - roll) if self.roll_enabled else 0.0

        # -------------------------------------------------------
        # PITCH: outer loop converts distance error → target pitch
        #        inner loop drives actual pitch to that target
        # -------------------------------------------------------
        distance = self.lidar.get_distance()
        angle = self.lidar.get_angle()

        target_pitch = 0.0
        target_roll = 0.0

        if distance is not None and angle is not None:
            theta = math.radians(angle)

            # Error: positive if obstacle is farther than desired,
            # negative if obstacle is too close
            distance_error = distance - self.reference_distance

            # Direction components
            front_component = math.cos(theta)
            right_component = math.sin(theta)

            # Desired pitch/roll based on obstacle direction
            target_pitch = self.kp_pitch_outer * distance_error * front_component
            target_roll  = self.kp_pitch_outer * distance_error * right_component

            # Safety clamps
            target_pitch = max(-self.max_pitch, min(self.max_pitch, target_pitch))
            target_roll  = max(-self.max_pitch, min(self.max_pitch, target_roll))
        else:
            target_pitch = 0.0
            target_roll = 0.0

        roll = self.get_roll()
        pitch = self.get_pitch()
        roll_correction = self.kp_roll * (target_roll - roll) if self.roll_enabled else 0.0
        pitch_correction = self.kp_pitch_inner * (target_pitch - pitch) if self.pitch_enabled else 0.0

        # -------------------------------------------------------
        # MOTOR MIXING: roll + pitch combined
        # M1 (front-right), M2 (back-left), M3 (front-left), M4 (back-right)
        # -------------------------------------------------------
        m1 = self.base_throttle - roll_correction - pitch_correction  # front-right
        m2 = self.base_throttle + roll_correction + pitch_correction  # back-left
        m3 = self.base_throttle + roll_correction - pitch_correction  # front-left
        m4 = self.base_throttle - roll_correction + pitch_correction  # back-right

        m1, m2, m3, m4 = [max(0.0, min(1.0, m)) for m in [m1, m2, m3, m4]]

        return m1, m2, m3, m4, roll_correction, pitch_correction, target_pitch, distance

    def run(self, duration_sec=10, loop_hz=10):
        interval = 1.0 / loop_hz
        end_time = time.time() + duration_sec

        print(f"  Reference distance: {self.reference_distance} cm")
        print(f"  {'Roll':>6} | {'Pitch':>6} | {'Dist':>7} | {'Err':>7} | {'TgtPitch':>8} | Motors")
        print("  " + "-" * 75)

        while time.time() < end_time:
            m1, m2, m3, m4, roll_corr, pitch_corr, target_pitch, distance = self.compute()

            roll  = self.get_roll()
            pitch = self.get_pitch()

            dist_str = f"{distance:.1f}cm" if distance is not None else "  N/A "
            err_str  = f"{(distance - self.reference_distance):+.1f}cm" if distance is not None else "  N/A "

            sys.stdout.write(
                f"\r  {roll:>6.2f}° | {pitch:>6.2f}° | {dist_str:>7} | {err_str:>7} | "
                f"{target_pitch:>7.2f}°  | "
                f"M1:{m1:.3f} M2:{m2:.3f} M3:{m3:.3f} M4:{m4:.3f}"
            )
            sys.stdout.flush()

            self.motors.actuator_test(1, m1, interval * 2)
            self.motors.actuator_test(2, m2, interval * 2)
            self.motors.actuator_test(3, m3, interval * 2)
            self.motors.actuator_test(4, m4, interval * 2)

            time.sleep(interval)

        for m in [1, 2, 3, 4]:
            self.motors.actuator_test(m, 0.0, 1)
        print("\nController stopped.")
