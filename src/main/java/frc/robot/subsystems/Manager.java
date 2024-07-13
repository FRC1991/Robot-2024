package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manager extends SubsystemBase implements CheckableSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  public Manager() {
    initialized = Intake.getInstance().getInitialized();
    initialized &= Pivot.getInstance().getInitialized();
    initialized &= Shooter.getInstance().getInitialized();
    initialized &= DriveSubsystem.getInstance().getInitialized();
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    Intake.getInstance().stop();
    Pivot.getInstance().stop();
    Shooter.getInstance().stop();
    DriveSubsystem.getInstance().stop();
  }

  /**
   *
   * @return Has the constructor executed successfully
   */
  @Override
  public boolean getInitialized() {
    return initialized;
  }

  /**
   *
   * @return Is the subsystem is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    status = Intake.getInstance().checkSubsystem();
    status &= Pivot.getInstance().checkSubsystem();
    status &= Shooter.getInstance().checkSubsystem();
    status &= DriveSubsystem.getInstance().checkSubsystem();

    return status;
  }
}
