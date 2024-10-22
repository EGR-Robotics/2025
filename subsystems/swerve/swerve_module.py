from wpimath.controller import PIDController

from wpimath.geometry import Rotation2d, Translation2d

from rev import CANSparkMax
from phoenix5.sensors import CANCoder

class SwerveModule:
    def __init__(self, drive_motor_id, steer_motor_id, can_coder_id, position: Translation2d, offset):
        # Initialize motors and sensors
        self.drive_motor = CANSparkMax(drive_motor_id, CANSparkMax.MotorType.kBrushless)
        self.steer_motor = CANSparkMax(steer_motor_id, CANSparkMax.MotorType.kBrushless)
        self.cancoder = CANCoder(can_coder_id)

        self.cancoder.configMagnetOffset(offset)

        # PID controller for steering
        self.steer_pid = PIDController(0, 0.0, 0.0)

        # Position of this module on the robot
        self.position = position

    def get_angle(self):
        # Get the absolute position of the swerve module's angle from the CANCoder
        return Rotation2d.fromDegrees(self.cancoder.getAbsolutePosition())

    def set_angle(self, target_angle: Rotation2d):
        # Get the current angle
        current_angle = self.get_angle()
        
        # Calculate the output from the PID controller to reach the desired angle
        output = self.steer_pid.calculate(current_angle.degrees(), target_angle.degrees())

        # Set the steer motor output
        self.steer_motor.set(output)

    def set_speed(self, speed):
        # Set the drive motor speed (between -1.0 and 1.0)
        self.drive_motor.set(speed)
