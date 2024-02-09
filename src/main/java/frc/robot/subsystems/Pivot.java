// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private CANSparkMax pivotMotor1, pivotMotor2;
  private AbsoluteEncoder pivotEncoder;

  public Pivot() {
    pivotMotor1 = new CANSparkMax(PivotConstants.kPivotMotor1Id, MotorType.kBrushless);
    pivotMotor2 = new CANSparkMax(PivotConstants.kPivotMotor2Id, MotorType.kBrushless);

    pivotEncoder = pivotMotor1.getAbsoluteEncoder(Type.kDutyCycle);

    pivotMotor1.setSoftLimit(SoftLimitDirection.kForward, 2);
    pivotMotor1.setSoftLimit(SoftLimitDirection.kForward, 2);
  }

  public void setPivot(double speed) {
    pivotMotor1.set(speed);
    pivotMotor2.set(-speed);
  }

  public double getAbsoluteEncoderValue() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
