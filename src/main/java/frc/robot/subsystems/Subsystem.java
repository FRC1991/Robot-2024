// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {

  public Subsystem() {}

  public abstract void stop();

  public abstract boolean checkSubsystem();

  public static boolean checkMotor(CANSparkMax motor, int CANId) {
    // Check CAN Id
    if(motor.getDeviceId() != CANId) {
      return false;
    }

    // Check motor temp.
    if(motor.getMotorTemperature() >= 25) {
      return false;
    }

    // Check last error/fault
    if(motor.getLastError() != REVLibError.kOk) {
      return false;
    }

    // We should only be using Brushless motors
    if(motor.getMotorType() != MotorType.kBrushless) {
      return false;
    }

    // Just in case
    // It should never enter the body of this if statement
    if(motor.getMotorType() != MotorType.fromId(CANId)) {
      return false;
    }

    return true;
  }
}
