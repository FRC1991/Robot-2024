// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {

  private CANSparkMax pivotMotor1, pivotMotor2;

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor1 = new CANSparkMax(PivotConstants.kPivotMotor1Id, MotorType.kBrushless);
    pivotMotor2 = new CANSparkMax(PivotConstants.kPivotMotor2Id, MotorType.kBrushless);

    //TODO find the actual limit for these
    pivotMotor1.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.kForwardLimit);
    pivotMotor2.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.kForwardLimit);

    //TODO find the actual limit for these
    pivotMotor1.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.kBackLimit);
    pivotMotor2.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.kBackLimit);
  }

  public void setPivot(double speed) {
    pivotMotor1.set(speed);
    pivotMotor2.set(-speed);
  }

  public double getEncoderPosition() {
    // Pivot motor 2 is subtracted because it is run in reverse
    return (pivotMotor1.getEncoder().getPosition() - pivotMotor2.getEncoder().getPosition()) / 2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
