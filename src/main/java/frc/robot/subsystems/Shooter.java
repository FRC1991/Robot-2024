// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import frc.utils.Utils;

public class Shooter extends SubsystemBase implements Subsystem {

  private boolean status = false;
  private boolean initialized = false;
  private CANSparkMax shooterMotor1, shooterMotor2;
  private static Shooter m_Instance;

  // Constructor is private to prevent multiple Shooters from being made
  private Shooter() {
    shooterMotor1 = new CANSparkMax(ShooterConstants.kShooterMotor1Id, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(ShooterConstants.kShooterMotor2Id, MotorType.kBrushless);

    // Ensures consistent setting whether or not I replaced a motor controller
    // mid competition *cough* *cough* getting all the wires caught
    // in a module's gears during a match at Waterbury *cough*
    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    // Motor is inverted because it faces the opposite direction of motor 2
    shooterMotor1.setInverted(true);

    // Setting idle mode to coast so the motors don't waste power stopping the fly wheels
    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterMotor2.setIdleMode(IdleMode.kCoast);

    // Saving the desired settings
    shooterMotor1.burnFlash();
    shooterMotor2.burnFlash();

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
   * @return The main Shooter object
   */
  public static Shooter getInstance() {
    if(m_Instance == null) {
      m_Instance = new Shooter();
    }
    return m_Instance;
  }

  /**
   *This should only be used through a {@link Command}, not directly accessed.
   * @param speed The desired speed to run the motors at.
   */
  public void setShooter(double speed) {
    speed = Utils.normalize(speed);
    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }

  public void setShooterRMP(double rpm) {
    //TODO test to find max rpm. 5000 max rpm is a guess
    rpm = rpm / 5000;
    rpm = Utils.normalize(rpm);
    setShooter(rpm);
  }

  /**
   * Stops movement in both motors
   */
  @Override
  public void stop() {
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
  }

  /**
   * Resets the relative encoder in both motors
   */
  public void zeroMotorEncoders() {
    shooterMotor1.getEncoder().setPosition(0);
    shooterMotor2.getEncoder().setPosition(0);
  }

  /**
   *
   * @return Is the subsystem is okay to operate
   */

  @Override
  public boolean checkSubsystem() {
    status = Utils.checkMotor(shooterMotor1, ShooterConstants.kShooterMotor1Id);
    status &= Utils.checkMotor(shooterMotor2, ShooterConstants.kShooterMotor2Id);
    status &= getInitialized();

    return status;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
