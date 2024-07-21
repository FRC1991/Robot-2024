package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveStates;
import frc.utils.Utils;
import frc.robot.subsystems.Intake.IntakeStates;
import frc.robot.subsystems.Pivot.PivotStates;
import frc.robot.subsystems.Shooter.ShooterStates;

public class Manager extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;

  private DoubleSupplier ty;

  private InterpolatingDoubleTreeMap pivotPosition;

  private ManagerStates desiredState, currentState = ManagerStates.IDLE;

  public Manager(DoubleSupplier tx, DoubleSupplier ty) {
    this.ty = ty;

    // TODO fill with experimental values and add more data points
    pivotPosition.put(0.0,0.0);
    pivotPosition.put(-1.0,-1.0);

    Pivot.getInstance().setAngleSupplier(() -> pivotPosition.get(ty.getAsDouble()));
    Swerve.getInstance().setAngleSupplier(tx);

    // All subsystems should initialize when calling getInstance()
    initialized = Intake.getInstance().getInitialized();
    initialized &= Pivot.getInstance().getInitialized();
    initialized &= Shooter.getInstance().getInitialized();
    initialized &= Swerve.getInstance().getInitialized();
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
    Intake.getInstance().update();
    Pivot.getInstance().update();
    Shooter.getInstance().update();
    Swerve.getInstance().update();

    switch(currentState) {
      case IDLE:
        break;
      case INTAKING:
        break;
      case DRIVE:
        break;
      case AIMMING:
        if(Pivot.getInstance().atSetpoint()
            && Swerve.getInstance().facingTarget()
            && Utils.getDistanceToTag(ty.getAsDouble()) <= 120
            && Shooter.getInstance().getState() == ShooterStates.SHOOTING) {
          setDesiredState(ManagerStates.SHOOTING);
        }
        break;
      case SHOOTING:
        break;
      case DEFENSE:
        break;
      case SUBWOOFER_AIMMING:
        if(Pivot.getInstance().atSetpoint() && Shooter.getInstance().getState() == ShooterStates.SHOOTING) {
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
        Swerve.getInstance().setDesiredState(SwerveStates.DRIVE);
        break;
      case SOURCE:
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

  /**
   * A Manager state integrates all of the other subsystem's states together
   * to create one cohesive state. This ensures that every subsystem is in
   * the correct state when performing an action. For example, when the
   * pivot is aiming, the shooter should be spinning up. When setting
   * the pivot state you may forget to spin up the shooter.
   *
   * <p>The "BROKEN" state which is present in all other subsystems
   * is not present here because it would mean that the whole
   * robot is broken. In that scenario the robot would be E-stopped
   * and no code would run after that anyway
  */
  public enum ManagerStates {
    IDLE,
    /** Picking up a note */
    INTAKING,
    /** Picking up a note at the source */
    SOURCE,
    /** Driving around the field */
    DRIVE,
    /** Aimming at the speaker with vision */
    AIMMING,
    /** Shooting a note with vision */
    SHOOTING,
    /** Locking the wheels in an X formation */
    DEFENSE,
    /** Aimming up against the Subwoofer. Also doubles as our feeding aimming over the stage */
    SUBWOOFER_AIMMING,
    /** Shooting at the Subwoofer. Also doubles as our feeding shot over the stage */
    SUBWOOFER_SHOOTING,
    /** Pushing a note out under our bumpers */
    OUTTAKING;
  }
}
