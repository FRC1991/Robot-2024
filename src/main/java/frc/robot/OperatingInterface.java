package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;

public class OperatingInterface {

  final Joystick driverJoytick;
  final XboxController auxController;

  public OperatingInterface() {
    // The driver's controller
    driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    // The aux's controller
    auxController = new XboxController(OIConstants.kAuxControllerPort);
  }  

  public double getDriveAxis(int axis) {
    return driverJoytick.getRawAxis(axis);
  }

  public boolean getDriveButton(int button) {
    return driverJoytick.getRawButton(button);
  }

  public double getAuxAxis(int axis) {
    return auxController.getRawAxis(axis);
  }

  public boolean getAuxButton(int button) {
    return auxController.getRawButton(button);
  }
}
