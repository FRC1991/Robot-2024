// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private CANSparkMax climberMotor1, climberMotor2;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor1 = new CANSparkMax(ClimberConstants.kClimberMotor1Id, MotorType.kBrushless);
    climberMotor2 = new CANSparkMax(ClimberConstants.kClimberMotor2Id, MotorType.kBrushless);

    climberMotor1.setIdleMode(IdleMode.kBrake);
    climberMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void setClimber(double speed) {
    climberMotor1.set(speed);
    climberMotor2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
