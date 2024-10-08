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
  private DoubleSupplier aimingAngle;
  private static Pivot m_Instance;
  private PivotStates desiredState, currentState = PivotStates.IDLE;

  // Constructor is private to prevent multiple instances from being made
  private Pivot() {
    ctrl = new PIDController(0.1, 0, 0);
    ctrl.setTolerance(3);

    pivotMotor1 = new CANSparkMax(PivotConstants.kPivotMotor1Id, MotorType.kBrushless);
    pivotMotor2 = new CANSparkMax(PivotConstants.kPivotMotor2Id, MotorType.kBrushless);

    // Ensures consistent setting whether or not I replaced a motor controller
    // mid competition *cough* *cough* getting all the wires caught
    // in a module's gears during a match at Waterbury *cough*
    pivotMotor1.restoreFactoryDefaults();
    pivotMotor2.restoreFactoryDefaults();

    // Motor is inverted because it faces the opposite direction of motor 2
    pivotMotor1.setInverted(true);

    // Setting idle mode to brake so the pivot won't move during collisions
    pivotMotor1.setIdleMode(IdleMode.kCoast);
    pivotMotor2.setIdleMode(IdleMode.kCoast);

    // Setting Encoder to return in degrees
    pivotMotor1.getEncoder().setPositionConversionFactor(4.44);
    pivotMotor2.getEncoder().setPositionConversionFactor(4.44);

    // Saving the desired settings
    pivotMotor1.burnFlash();
    pivotMotor2.burnFlash();

    zeroMotorEncoders();

    initialized = true;
  }

  /**
   * @return Has the constructor been executed
   */
  @Override
  public boolean getInitialized() {
    return initialized && aimingAngle != null;
  }

  /**
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
    // The angle is turned back into motor rotations so I don't have to retune the PID controller
    setSpeed(ctrl.calculate(getEncoderPosition()/4.44, angle/4.44));
  }

  /**
   * @return True, if the pivot angle is within half a degree of the target
   * <li> False, if the pivot angle is not withing the tolerance.
   */
  public boolean atSetpoint() {
    return ctrl.atSetpoint();
  }

  /**
   * @param getter A method to get the desired vertical angle for shooting
   */
  public void setAngleSupplier(DoubleSupplier getter) {
    aimingAngle = getter;
  }

  /**
   * @return The average encoder value between both motors internal relative encoders in degrees
   */
  public double getEncoderPosition() {
    // Pivot motor 2 is subtracted because it is run in reverse
    return (pivotMotor1.getEncoder().getPosition() + pivotMotor2.getEncoder().getPosition()) / 2;
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
   * @return Is the subsystem is okay to operate
   */
  @Override
  public boolean checkSubsystem() {
    status = Utils.checkMotor(pivotMotor1, PivotConstants.kPivotMotor1Id);
    status &= Utils.checkMotor(pivotMotor2, PivotConstants.kPivotMotor2Id);
    status &= getInitialized();
    status &= currentState != PivotStates.BROKEN;

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
        updateSpeed(aimingAngle.getAsDouble());
        break;
      case STORED:
        updateSpeed(0);
        break;
      case SETPOINT:
        // From OG motor rotation values multiplied by (1/81)*360
        updateSpeed(29.6);

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
      case SETPOINT:
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
    if(this.desiredState != state /*&& this.currentState != PivotStates.BROKEN*/) {
      desiredState = state;
      handleStateTransition();
    }
  }

  /**
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
    /** Aims with the aimingAngle Supplier */
    AIMING,
    /** Uses the aiming method with 0 degree angle */
    STORED,
    /** Aims to the subwoofer angle which is also used to shoot over the stage */
    SETPOINT;
  }
}
