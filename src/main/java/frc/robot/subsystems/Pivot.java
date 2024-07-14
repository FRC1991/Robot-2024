// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.PivotConstants;
import frc.utils.Utils;

public class Pivot extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;
  private CANSparkMax pivotMotor1, pivotMotor2;
  private PIDController ctrl;
  private DoubleSupplier aimmingAngle;
  private static Pivot m_Instance;
  private PivotStates desiredState, currentState = PivotStates.IDLE;

  // Constructor is private to prevent multiple instances from being made
  private Pivot() {
    // TODO Retune PID now that the encoder is in degrees, not native units
    ctrl = new PIDController(0.1, 0, 0);
    ctrl.setTolerance(0.02);

    pivotMotor1 = new CANSparkMax(PivotConstants.kPivotMotor1Id, MotorType.kBrushless);
    pivotMotor2 = new CANSparkMax(PivotConstants.kPivotMotor2Id, MotorType.kBrushless);

    // Ensures consistent setting whether or not I replaced a motor controller
    // mid competition *cough* *cough* getting all the wires caught
    // in a module's gears during a match at Waterbury *cough*
    pivotMotor1.restoreFactoryDefaults();
    pivotMotor2.restoreFactoryDefaults();

    // Motor is inverted because it faces the opposite direction of motor 1
    pivotMotor2.setInverted(true);

    // Setting idle mode to brake so the pivot won't move during collisions
    pivotMotor1.setIdleMode(IdleMode.kBrake);
    pivotMotor2.setIdleMode(IdleMode.kBrake);

    // Setting Encoder to return in degrees
    pivotMotor1.getEncoder().setPositionConversionFactor((1/81)*360);
    pivotMotor2.getEncoder().setPositionConversionFactor((1/81)*360);

    // Saving the desired settings
    pivotMotor1.burnFlash();
    pivotMotor2.burnFlash();

    zeroMotorEncoders();

    setDesiredState(PivotStates.STORED);

    initialized = true;
  }

  /**
   *
   * @return Has the constructor been executed
   */
  @Override
  public boolean getInitialized() {
    return initialized;
  }

  /**
   *
   * @return The main Pivot object
   */
  public static Pivot getInstance() {
    if(m_Instance == null) {
      m_Instance = new Pivot();
    }
    return m_Instance;
  }

  /**
   *This should only be used through a {@link Command}, not directly accessed.
   * @param speed The desired speed to run the motors at.
   */
  public void setSpeed(double speed) {
    speed = Utils.normalize(speed);
    pivotMotor1.set(speed);
    pivotMotor2.set(speed);
  }

  /**
   * Uses this subsystem's PID controller to update the motor speed
   * based off of the current position.
   * @param angle Desired angle to move the pivot to.
   */
  public void updateSpeed(double angle) {
    setSpeed(ctrl.calculate(getEncoderPosition(), angle));
  }

  /**
   *
   * @param getter A method to get the desired angle for shooting
   */
  public void setAngleSupplier(DoubleSupplier getter) {
    aimmingAngle = getter;
  }

  /**
   *
   * @return The average encoder value between both motors internal relative encoders
   */
  public double getEncoderPosition() {
    // Pivot motor 2 is subtracted because it is run in reverse
    return (pivotMotor1.getEncoder().getPosition() - pivotMotor2.getEncoder().getPosition()) / 2;
  }

  /**
   * Resets the relative encoder in both motors
   */
  public void zeroMotorEncoders() {
    pivotMotor1.getEncoder().setPosition(0);
    pivotMotor2.getEncoder().setPosition(0);
  }

  /**
   * Stops movement in all motors
   */
  @Override
  public void stop() {
    pivotMotor1.stopMotor();
    pivotMotor2.stopMotor();
  }

  /**
   *
   * @return Is the subsystem is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    status = Utils.checkMotor(pivotMotor1, PivotConstants.kPivotMotor1Id);
    status &= Utils.checkMotor(pivotMotor2, PivotConstants.kPivotMotor2Id);
    status &= getInitialized();

    return status;
  }

  /**
   * Updates any information the subsystem needs
   */
  @Override
  public void update() {
    switch (currentState) {
      case IDLE:
        break;
      case BROKEN:
        break;
      case AIMING:
        updateSpeed(aimmingAngle.getAsDouble());
        break;
      case STORED:
        updateSpeed(0);
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(PivotStates.BROKEN);
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
      case AIMING:
        break;
      case STORED:
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
  public void setDesiredState(PivotStates state) {
    if(this.desiredState != state && this.currentState != PivotStates.BROKEN) {
      desiredState = state;
      handleStateTransition();
    }
  }

  /**
   *
   * @return The current state of the subsystem
   */
  public PivotStates getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum PivotStates {
    IDLE,
    BROKEN,
    AIMING, // Aims with the aimmingAngle Supplier
    STORED, // Uses the aimming method with 0 degree angle
  }
}
