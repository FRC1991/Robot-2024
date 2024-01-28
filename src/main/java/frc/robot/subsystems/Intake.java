// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor1, intakeMotor2;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotor1Id, MotorType.kBrushless);
    intakeMotor2 = new CANSparkMax(IntakeConstants.kIntakeMotor2Id, MotorType.kBrushless);
  }

  public CANSparkMax getIntakeMotor1() {
    return intakeMotor1;
  }

  public CANSparkMax getIntakeMotor2() {
    return intakeMotor2;
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }

  public RelativeEncoder getEncoder1() {
    return intakeMotor1.getEncoder();
  }

  public RelativeEncoder getEncoder2() {
    return intakeMotor2.getEncoder();
  }

  public void resetEncoders() {
    intakeMotor1.getEncoder().setPosition(0);
    intakeMotor2.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
