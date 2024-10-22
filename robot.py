import wpilib

from subsystems.swerve.swerve import SwerveDrive

from constants import DRIVER_CONTROLLER_PORT

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Initialize swerve drive system
        self.swerve_drive = SwerveDrive()

        # Driver controls (assume joystick is plugged into port 0)
        self.driver_controller = wpilib.XboxController(DRIVER_CONTROLLER_PORT)
        

    def teleopPeriodic(self):
        # Get joystick inputs for driving
        fwd = -self.driver_controller.getLeftY()
        
        # Account for stick drift
        strafe = self.driver_controller.getLeftX()
        rot = self.driver_controller.getRightY()

        # Pass the inputs to the swerve drive system
        self.swerve_drive.drive(fwd, strafe, rot)

if __name__ == '__main__':
    wpilib.run(MyRobot)
