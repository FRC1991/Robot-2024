// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.utils.Utils;

public class Intake extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;
  private CANSparkMax intakeMotor1, intakeMotor2;
  private static Intake m_Instance;
  private IntakeStates desiredState, currentState = IntakeStates.IDLE;

  // Constructor is private to prevent multiple instances from being made
  private Intake() {
    intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotor1Id, MotorType.kBrushless);
    intakeMotor2 = new CANSparkMax(IntakeConstants.kIntakeMotor2Id, MotorType.kBrushless);

    // Ensures consistent setting whether or not I replaced a motor controller
    // mid competition *cough* *cough* getting all the wires caught
    // in a module's gears during a match at Waterbury *cough*
    intakeMotor1.restoreFactoryDefaults();
    intakeMotor2.restoreFactoryDefaults();

    // Motor is inverted because it faces the opposite direction of motor 1
    intakeMotor2.setInverted(true);

    // Setting idle mode to coast so the motors don't waste power stopping the intake and belts
    intakeMotor1.setIdleMode(IdleMode.kCoast);
    intakeMotor2.setIdleMode(IdleMode.kCoast);

    // Saving the desired settings
    intakeMotor1.burnFlash();
    intakeMotor2.burnFlash();

    zeroMotorEncoders();

    initialized = true;
  }

  /**
   * @return Has the constructor executed successfully
   */
  @Override
  public boolean getInitialized() {
    return initialized;
  }

  /**
   * @return The main Shooter object
   */
  public static Intake getInstance() {
    if(m_Instance == null) {
      m_Instance = new Intake();
    }
    return m_Instance;
  }

  /**
   *This should only be used through a {@link Command}, not directly accessed.
   * @param speed The desired speed to run the motors at.
   */
  public void setSpeed(double speed) {
    speed = Utils.normalize(speed);
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    intakeMotor1.stopMotor();
    intakeMotor2.stopMotor();
  }

  /**
   * Resets the relative encoder in both motors
   */
  public void zeroMotorEncoders() {
    intakeMotor1.getEncoder().setPosition(0);
    intakeMotor2.getEncoder().setPosition(0);
  }

  /**
   * @return Is the subsystem is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    status = Utils.checkMotor(intakeMotor1, IntakeConstants.kIntakeMotor1Id);
    status &= Utils.checkMotor(intakeMotor2, IntakeConstants.kIntakeMotor2Id);
    status &= getInitialized();

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
      case BROKEN:
        break;
      case INTAKING:
        break;
      case OUTTAKING:
        break;
      case FEEDING:
        break;
      case LOADED:
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(IntakeStates.BROKEN);
    }
  }

  /**
   * Handles moving from one state to another
   */
  @Override
  public void handleStateTransition() {
    switch(desiredState) {
      case IDLE:
        setSpeed(0);
        break;
      case BROKEN:
        stop();
        break;
      case INTAKING:
        setSpeed(0.8);
        break;
      case OUTTAKING:
        setSpeed(-0.6);
        break;
      case FEEDING:
        setSpeed(1);
        break;
      case LOADED:
        setSpeed(0);
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
  public void setDesiredState(IntakeStates state) {
    if(this.desiredState != state && this.currentState != IntakeStates.BROKEN) {
      desiredState = state;
      handleStateTransition();
    }
  }

  /**
   * @return The current state of the subsystem
   */
  public IntakeStates getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum IntakeStates {
    IDLE,
    BROKEN,
    /** Picking up note from the ground */
    INTAKING,
    /** Spitting a note out, under the bumpers */
    OUTTAKING,
    /** Advancing a note into the shooter */
    FEEDING,
    /** Holding a note in the belts */
    LOADED;
  }
}
