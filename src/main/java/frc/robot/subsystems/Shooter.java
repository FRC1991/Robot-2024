// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax shooterNeo1, shooterNeo2;

  public Shooter() {
    //TODO change device IDs
    shooterNeo1 = new CANSparkMax(13, MotorType.kBrushless);
    shooterNeo2 = new CANSparkMax(14, MotorType.kBrushless);

    shooterNeo2.follow(shooterNeo1);
  }

  private void setShooter(double speed) {
    shooterNeo1.set(speed);
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
