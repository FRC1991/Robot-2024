package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveStates;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Manager extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private ManagerStates desiredState, currentState = ManagerStates.IDLE;

  public Manager(DoubleSupplier verticalOffset, DoubleSupplier horizontalOffset) {
    initialized = Intake.getInstance().getInitialized();
    initialized &= Pivot.getInstance().getInitialized();
    initialized &= Shooter.getInstance().getInitialized();
    initialized &= Swerve.getInstance().getInitialized();

    ShuffleboardTab diagnostics = Shuffleboard.getTab("diagnostics");

    diagnostics.addBoolean("Manager initialized", this::getInitialized);
    diagnostics.addBoolean("Intake initialized", Intake.getInstance()::getInitialized);
    diagnostics.addBoolean("Pivot initialized", Pivot.getInstance()::getInitialized);
    diagnostics.addBoolean("Shooter initialized", Shooter.getInstance()::getInitialized);
    diagnostics.addBoolean("Swerve initialized", Swerve.getInstance()::getInitialized);

    diagnostics.addBoolean("Manager operable", this::checkSubsystem);
    diagnostics.addBoolean("Intake operable", Intake.getInstance()::checkSubsystem);
    diagnostics.addBoolean("Pivot operable", Pivot.getInstance()::checkSubsystem);
    diagnostics.addBoolean("Shooter operable", Shooter.getInstance()::checkSubsystem);
    diagnostics.addBoolean("Swerve operable", Swerve.getInstance()::checkSubsystem);
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    Intake.getInstance().stop();
    Pivot.getInstance().stop();
    Shooter.getInstance().stop();
    Swerve.getInstance().stop();
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
    status &= Swerve.getInstance().checkSubsystem();

    return status;
  }

  /**
   * Updates any information the subsystem needs
   */
  @Override
  public void update() {
    switch(currentState) {
      case IDLE:
        break;
      case INTAKING:
        break;
      case DRIVE:
        break;
      case AIMMING:
        if(Pivot.getInstance().atSetpoint() && Swerve.getInstance().facingTarget()) {
          setDesiredState(ManagerStates.SHOOTING);
        }
        break;
      case SHOOTING:
        break;
      case DEFENSE:
        break;
      case SUBWOOFER_AIMMING:
        if(Pivot.getInstance().atSetpoint()) {
          setDesiredState(ManagerStates.SUBWOOFER_SHOOTING);
        }
        break;
      case SUBWOOFER_SHOOTING:
        break;
      case OUTTAKING:
        break;

      default:
        break;
    }
  }

  /**
   * Handles moving from one state to another. Also changes
   * the state of all subsystems to their respective states
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        Intake.getInstance().setDesiredState(IntakeStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.IDLE);
        Shooter.getInstance().setDesiredState(ShooterStates.IDLE);
        Swerve.getInstance().setDesiredState(SwerveStates.IDLE);
        break;
      case INTAKING:
        Intake.getInstance().setDesiredState(IntakeStates.INTAKING);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Shooter.getInstance().setDesiredState(ShooterStates.IDLE);
        Swerve.getInstance().setDesiredState(SwerveStates.PICKUP);
        break;
      case DRIVE:
        Intake.getInstance().setDesiredState(IntakeStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Shooter.getInstance().setDesiredState(ShooterStates.IDLE);
        Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
        break;
      case AIMMING:
        Intake.getInstance().setDesiredState(IntakeStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.AIMMING);
        Shooter.getInstance().setDesiredState(ShooterStates.SPINNING_UP);
        Swerve.getInstance().setDesiredState(SwerveStates.AIMMING);
        break;
      case SHOOTING:
        Intake.getInstance().setDesiredState(IntakeStates.FEEDING);
        Pivot.getInstance().setDesiredState(PivotStates.AIMMING);
        Shooter.getInstance().setDesiredState(ShooterStates.SHOOTING);
        Swerve.getInstance().setDesiredState(SwerveStates.AIMMING);
        break;
      case DEFENSE:
        Intake.getInstance().setDesiredState(IntakeStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Shooter.getInstance().setDesiredState(ShooterStates.IDLE);
        Swerve.getInstance().setDesiredState(SwerveStates.LOCKED);
        break;
      case SUBWOOFER_AIMMING:
        Intake.getInstance().setDesiredState(IntakeStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.SETPOINT);
        Shooter.getInstance().setDesiredState(ShooterStates.SPINNING_UP);
        Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
        break;
      case SUBWOOFER_SHOOTING:
        Intake.getInstance().setDesiredState(IntakeStates.FEEDING);
        Pivot.getInstance().setDesiredState(PivotStates.SETPOINT);
        Shooter.getInstance().setDesiredState(ShooterStates.SHOOTING);
        Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
        break;
      case OUTTAKING:
        Intake.getInstance().setDesiredState(IntakeStates.OUTTAKING);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Shooter.getInstance().setDesiredState(ShooterStates.IDLE);
        Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
        break;

      default:
        break;
    }

    currentState = desiredState;
  }

  /**
   * Sets the desired state of the subsystem
   * @param state Desired state
   */
  public void setDesiredState(ManagerStates state) {
    if(this.desiredState != state) {
      desiredState = state;
      handleStateTransition();
    }
  }

  /**
   *
   * @return The current state of the subsystem
   */
  public ManagerStates getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // TODO Add javadoc comments on ALL subsystem states
  /* The BROKEN state which is present in all other subsystems
   * is not present here because it would mean that the whole
   * robot is broken. In that scenario the robot would be E-stopped
   * and no code would run after that anyway
  */
  public enum ManagerStates {
    IDLE,
    INTAKING, // Picking up a note at the source
    DRIVE, // Driving around the field
    AIMMING, // Aimming at the speaker with vision
    SHOOTING, // Shooting a note out with vision
    DEFENSE, // Locking the wheels in an X
    SUBWOOFER_AIMMING, // Shooting over the stage. Also doubles as our subwoofer shot
    SUBWOOFER_SHOOTING,
    OUTTAKING; // Pushing a note out under our bumpers
  }
}
