package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class OperatingInterface {

  // The Driver's joystick
  public final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  // The Auxillary driver's controller
  public final XboxController auxController = new XboxController(OIConstants.kAuxControllerPort);

  public Trigger auxXButton = new Trigger(() -> auxController.getXButton());
  public Trigger auxAButton = new Trigger(() -> auxController.getAButton());
  public Trigger auxBButton = new Trigger(() -> auxController.getBButton());
  public Trigger auxYButton = new Trigger(() -> auxController.getYButton());
  public Trigger auxLeftBumper = new Trigger(() -> auxController.getLeftBumper());
  public Trigger auxRightBumper = new Trigger(() -> auxController.getRightBumper());
  public Trigger auxStartButton = new Trigger(() -> auxController.getStartButton());
  public Trigger auxBackButton = new Trigger(() -> auxController.getBackButton());

  public OperatingInterface() {}
}
