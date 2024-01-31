// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.PID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAnglePID extends PIDCommand {

  // private DriveSubsystem m_DriveSubsystem;
  // private double targetAngle;

  /**
   * Smoothly turns to the target angle via the shortest path. This command does not end and will continuously
   * attempt to turn to the target angle even if disturbed or blocked.
   * @param targetAngle Angle the robot will turn to. Must be between 0 and 360
   * @param driveSubsystem The main drive train of the robot
   */
  public TurnToAnglePID(double targetAngle, DriveSubsystem driveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.009, 0, 0),
        // This should return the measurement
        () -> driveSubsystem.getHeadingTurnToAngle(targetAngle),
        // This should return the setpoint (can also be a constant)
        () -> targetAngle,
        // This uses the output
        output -> {
          if(output > 1) {
            output = 1;
          } else if (output < 0.01) {
            output = 0;
          }
          driveSubsystem.drive(0, 0, output, false, false);
        });
    
    // this.targetAngle = targetAngle;
    // m_DriveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(targetAngle - m_DriveSubsystem.getHeading()) <= 5) {
    //   System.out.println("leaving TurnToAnglePID " + targetAngle);
    //   return true;
    // } else {
    //   return false;
    // }

    return false;
  }
}
