// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;

  /** Creates a new Intake. */
  public Intake(int intakeMotorId) {
    intakeMotor = new CANSparkMax(intakeMotorId, MotorType.kBrushless);
  }

  public CANSparkMax getIntakeMotor() {
    return intakeMotor;
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public RelativeEncoder getEncoder() {
    return intakeMotor.getEncoder();
  }

  public void resetEncoder() {
    intakeMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
