package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    pivotPosition = new InterpolatingDoubleTreeMap();
    garageShots();

    Shuffleboard.getTab("Vision").addDouble("desired angle", () -> pivotPosition.get(ty.getAsDouble()));

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
   * To clarify the naming of this function, I am currently in my garage with the robot.
   * All of these data points are from the robot on a tote on a dolly aiming at an apriltag
   * which I taped to my garage wall. I then used these measurements a calculate the outputs with
   * the desmos graph I made because I forgot to bring a note to test.
   */
  private void garageShots() {
    pivotPosition.put(10.72, 32.62);
    pivotPosition.put(10.31, 32.35);
    pivotPosition.put(9.67, 31.92);
    pivotPosition.put(9.15, 31.57);
    pivotPosition.put(8.50, 31.13);
    pivotPosition.put(7.93, 30.73);
    pivotPosition.put(6.71, 29.89);
    pivotPosition.put(5.08, 28.76);
    pivotPosition.put(4.41, 28.28);
    pivotPosition.put(3.52, 27.65);
    pivotPosition.put(2.44, 26.88);
    pivotPosition.put(1.22, 25.98);
    pivotPosition.put(0.00, 25.08);
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
   * @return Is the robot is okay to operate
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
        setDesiredState(ManagerStates.DRIVE);
        break;
      case INTAKING:
        break;
      case DRIVE:
        break;
      case AIMING:
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
      case SUBWOOFER_AIMING:
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
      case AIMING:
        Intake.getInstance().setDesiredState(IntakeStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.AIMING);
        Shooter.getInstance().setDesiredState(ShooterStates.SPINNING_UP);
        Swerve.getInstance().setDesiredState(SwerveStates.AIMING);
        break;
      case SHOOTING:
        Intake.getInstance().setDesiredState(IntakeStates.FEEDING);
        Pivot.getInstance().setDesiredState(PivotStates.AIMING);
        Shooter.getInstance().setDesiredState(ShooterStates.SHOOTING);
        Swerve.getInstance().setDesiredState(SwerveStates.AIMING);
        break;
      case DEFENSE:
        Intake.getInstance().setDesiredState(IntakeStates.IDLE);
        Pivot.getInstance().setDesiredState(PivotStates.STORED);
        Shooter.getInstance().setDesiredState(ShooterStates.IDLE);
        Swerve.getInstance().setDesiredState(SwerveStates.LOCKED);
        break;
      case SUBWOOFER_AIMING:
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
    AIMING,
    /** Shooting a note with vision */
    SHOOTING,
    /** Locking the wheels in an X formation */
    DEFENSE,
    /** Aimming up against the Subwoofer. Also doubles as our feeding aiming over the stage */
    SUBWOOFER_AIMING,
    /** Shooting at the Subwoofer. Also doubles as our feeding shot over the stage */
    SUBWOOFER_SHOOTING,
    /** Pushing a note out under our bumpers */
    OUTTAKING;
  }
}
