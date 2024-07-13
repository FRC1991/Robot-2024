package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class OperatingInterface {

  // The Driver's joystick
  public static final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  // The Auxillary driver's controller
  public static final XboxController auxController = new XboxController(OIConstants.kAuxControllerPort);

  // Creating Triggers based off auxController's buttons for ease of use
  public static Trigger auxXButton = new Trigger(() -> auxController.getXButton());
  public static Trigger auxAButton = new Trigger(() -> auxController.getAButton());
  public static Trigger auxBButton = new Trigger(() -> auxController.getBButton());
  public static Trigger auxYButton = new Trigger(() -> auxController.getYButton());
  public static Trigger auxLeftBumper = new Trigger(() -> auxController.getLeftBumper());
  public static Trigger auxRightBumper = new Trigger(() -> auxController.getRightBumper());
  public static Trigger auxStartButton = new Trigger(() -> auxController.getStartButton());
  public static Trigger auxBackButton = new Trigger(() -> auxController.getBackButton());

  // A half second rumble on the Xbox controller
  public static void rumbleAuxController() {
    Thread rumble = new Thread(() -> {
      auxController.setRumble(RumbleType.kBothRumble, .9);
      Timer.delay(0.5);
      auxController.setRumble(RumbleType.kBothRumble, 0);
    });
    rumble.start();
  }

  public OperatingInterface() {}
}
