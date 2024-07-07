// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.PivotConstants;
import frc.utils.Utils;

public class Pivot extends SubsystemBase implements CheckableSubsystem {

  private boolean status = false;
  private boolean initialized = false;
  private CANSparkMax pivotMotor1, pivotMotor2;
  private static Pivot m_Instance;

  // Constructor is private to prevent multiple Pivots from being made
  private Pivot() {
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

    // Saving the desired settings
    pivotMotor1.burnFlash();
    pivotMotor2.burnFlash();

    zeroMotorEncoders();

    initialized = true;
  }

  /**
   *
   * @return Has the constructor been executed
   */
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
  public void setPivot(double speed) {
    speed = Utils.normalize(speed);
    pivotMotor1.set(speed);
    pivotMotor2.set(speed);
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
   * Stops movement in both motors
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
