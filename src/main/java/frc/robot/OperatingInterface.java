package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;

public class OperatingInterface {
  private OperatingInterface() {}

  // The Driver's joystick
  public static final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
}
