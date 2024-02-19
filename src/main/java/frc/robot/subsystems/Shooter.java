// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterNeo1, shooterNeo2;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterNeo1 = new CANSparkMax(ShooterConstants.kShooterMotor1Id, MotorType.kBrushless);
    shooterNeo2 = new CANSparkMax(ShooterConstants.kShooterMotor2Id, MotorType.kBrushless);
  }

  public void setShooter(double speed) {
    shooterNeo1.set(speed);
    shooterNeo2.set(-speed);
  }

  public void setShooterRMP(double rpm) {
    //TODO test to find max rpm. 5000 max rpm is a guess 
    rpm = rpm / 5000;

    if(rpm > 1) {
      rpm = 1;
    } else if(rpm < 0) {
      rpm = 0;
    }

    setShooter(rpm);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
