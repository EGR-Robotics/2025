from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds

from phoenix5.sensors import Pigeon2

from .swerve_module import SwerveModule

from constants import (
    FR_DRIVE_MOTOR_ID,
    FR_STEER_MOTOR_ID,
    FR_CAN_CODER_ID,
    FR_OFFSET,
    FL_DRIVE_MOTOR_ID,
    FL_STEER_MOTOR_ID,
    FL_CAN_CODER_ID,
    FL_OFFSET,
    BR_DRIVE_MOTOR_ID,
    BR_STEER_MOTOR_ID,
    BR_CAN_CODER_ID,
    BR_OFFSET,
    BL_DRIVE_MOTOR_ID,
    BL_STEER_MOTOR_ID,
    BL_CAN_CODER_ID,
    BL_OFFSET,
    GYRO_ID
)

class SwerveDrive:
    def __init__(self):
        # Define the positions of each swerve module relative to the robot center
        self.modules = [
            # Front-left module
            SwerveModule(
                FL_DRIVE_MOTOR_ID,
                FL_STEER_MOTOR_ID,
                FL_CAN_CODER_ID,
                Translation2d(0.5, -0.5),
                FR_OFFSET
            ),  
            # Front-right module
            SwerveModule(
                FR_DRIVE_MOTOR_ID,
                FR_STEER_MOTOR_ID,
                FR_CAN_CODER_ID,
                Translation2d(0.5, 0.5),
                FL_OFFSET
            ),
            # Back-left module
            SwerveModule(
                BL_DRIVE_MOTOR_ID,
                BL_STEER_MOTOR_ID,
                BL_CAN_CODER_ID,
                Translation2d(-0.5, 0.5),
                BL_OFFSET
            ),  
            # Back-right module
            SwerveModule(
                BR_DRIVE_MOTOR_ID,
                BR_STEER_MOTOR_ID,
                BR_CAN_CODER_ID,
                Translation2d(-0.5, -0.5),
                BR_OFFSET
            ),
        ]

        # Create a kinematics object for converting between chassis speeds and individual module states
        self.kinematics = SwerveDrive4Kinematics(
            self.modules[0].position,
            self.modules[1].position,
            self.modules[2].position,
            self.modules[3].position
        )

        # Gyro (assume gyro is connected via WPILib)
        self.gyro = Pigeon2(GYRO_ID)

    def drive(self, fwd, strafe, rot):
        # Get the desired chassis speeds from the inputs
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fwd,
            strafe,
            rot,
            Rotation2d.fromDegrees(self.gyro.getYaw())
        )

        # Convert the chassis speeds to module states
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)

        # Loop over each swerve module, set its speed and angle
        for module, state in zip(self.modules, module_states):
            module.set_speed(state.speed)
            module.set_angle(state.angle)
