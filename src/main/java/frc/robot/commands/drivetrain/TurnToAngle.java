// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends Command {

  private double angle, currentHeading;
  private DriveSubsystem m_DriveSubsystem;
  /** 
   * Minimizes difference between the robot heading and the target angle
   * The command ends when the difference is below the tolerance
   * 
   * @param angle The angle between the target and the current heading of the robot
   * @param driveSubsystem The drive subsystem of the robot
  */
  public TurnToAngle(double angle, DriveSubsystem driveSubsystem) {
  public TurnToAngle(double angle, double tolerance, DriveSubsystem driveSubsystem) {
    this.angle = angle;
    this.tolerance = tolerance;
    this.m_DriveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHeading = m_DriveSubsystem.getHeading();

    //optimizing angle for continous output
    if(currentHeading > (angle + 180)) {
      currentHeading -= 360;
    } else if(currentHeading < (angle - 180)) {
      currentHeading += 360;
    }
    
    //driving of the motors
    if(currentHeading < 180) {
      m_DriveSubsystem.drive(0, 0, -0.2, false, true);
    } else if(currentHeading > 180) {
      m_DriveSubsystem.drive(0, 0, 0.2, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(angle - m_DriveSubsystem.getHeading()) <= tolerance) {
      return true;
    } else {
      return false;
    }
  }
}
