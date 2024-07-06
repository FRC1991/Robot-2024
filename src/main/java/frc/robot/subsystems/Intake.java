// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.utils.Utils;

public class Intake extends Subsystem {

  private CANSparkMax intakeMotor1, intakeMotor2;
  private static Intake m_Instance;

  // Constructor is private to prevent multiple Shooters from being made
  private Intake() {
    intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotor1Id, MotorType.kBrushless);
    intakeMotor2 = new CANSparkMax(IntakeConstants.kIntakeMotor2Id, MotorType.kBrushless);

    // Motor is inverted because it faces the opposite direction of motor 1
    intakeMotor2.setInverted(true);

    // Setting idle mode to coast so the motors don't waste power stopping the intake and belts
    intakeMotor1.setIdleMode(IdleMode.kCoast);
    intakeMotor2.setIdleMode(IdleMode.kCoast);

    // Saving the desired settings
    intakeMotor1.burnFlash();
    intakeMotor2.burnFlash();

    zeroMotorEncoders();
  }

  /**
   *
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
  public void setIntakeSpeed(double speed) {
    speed = Utils.normalize(speed);
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }

  /**
   * Stops movement in both motors
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
   *
   * @return Is the subsystem is okay to operate
   */

  @Override
  public boolean checkSubsystem() {
    boolean status = false;
    status = Utils.checkMotor(intakeMotor1, IntakeConstants.kIntakeMotor1Id);
    status &= Utils.checkMotor(intakeMotor2, IntakeConstants.kIntakeMotor2Id);

    return status;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
