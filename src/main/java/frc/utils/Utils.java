package frc.utils;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class Utils {
    /**
     * All speeds that a motor is set to should be between -1 and 1.
     * This method ensures that all values will be safe for a motor to run at.
     * @param input The desired speed of a motor
     * @return The desired speed normalized between [-1, 1]
     */
    public static double normalize(double input) {
        if(input > 1) {
            return 1;
        } else if(input < -1) {
            return -1;
        } else if(Double.isNaN(input) || Double.isInfinite(input)) {
            return 0;
        }
        return input;
    }

    /**
     * Tests that the motor has a matching CAN Id, reasonable temperature, has not
     * thrown any errors, and is of MotorType.kBrushless
     * @param motor The CANSparkMax to be tested
     * @param CANId The desired CAN Id for this motor
     * @return Has the motor passed all of the tests?
     */
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
