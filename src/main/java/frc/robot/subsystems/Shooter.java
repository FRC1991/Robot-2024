// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TeleopConstants;
import frc.utils.Utils;

public class Shooter extends SubsystemBase implements CheckableSubsystem, StateSubsystem {

  private boolean status = false;
  private boolean initialized = false;
  private double spinUpStart;
  private CANSparkMax shooterMotor1, shooterMotor2;
  private static Shooter m_Instance;
  private ShooterStates desiredState, currentState = ShooterStates.IDLE;

  // Constructor is private to prevent multiple instances from being made
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
   * @return Has the constructor been executed
   */
  @Override
  public boolean getInitialized() {
    return initialized;
  }

  /**
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
  public void setSpeed(double speed) {
    speed = Utils.normalize(speed);
    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }

  /**
   * Stops movement in all motors
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
   * @return Is the subsystem is okay to operate
   */

  @Override
  public boolean checkSubsystem() {
    status = Utils.checkMotor(shooterMotor1, ShooterConstants.kShooterMotor1Id);
    status &= Utils.checkMotor(shooterMotor2, ShooterConstants.kShooterMotor2Id);
    status &= getInitialized();
    status &= currentState != ShooterStates.BROKEN;

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
      case SHOOTING:
        break;
      case SPINNING_UP:
        if(spinUpStart + 0.6 <= Timer.getFPGATimestamp()) {
          setDesiredState(ShooterStates.SHOOTING);
        }
        break;

      default:
        break;
    }

    if(!checkSubsystem()) {
      setDesiredState(ShooterStates.BROKEN);
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
      case SHOOTING:
        setSpeed(TeleopConstants.kShooterSpeed);
        break;
      case SPINNING_UP:
        spinUpStart = Timer.getFPGATimestamp();
        setSpeed(TeleopConstants.kShooterSpeed);
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
  public void setDesiredState(ShooterStates state) {
    if(this.desiredState != state /*&& this.currentState != ShooterStates.BROKEN*/) {
      desiredState = state;
      handleStateTransition();
    }
  }

  /**
   * @return The current state of the subsystem
   */
  public ShooterStates getState() {
    return currentState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum ShooterStates {
    IDLE,
    BROKEN,
    /** At speed and ready to shoot a note */
    SHOOTING,
    /** Spinning up to the desired speed */
    SPINNING_UP;
  }
}
