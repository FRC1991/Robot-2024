// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.utils.SwerveUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends Command {

  private double angle;
  private DriveSubsystem m_DriveSubsystem;
  /** 
   * Creates a command that minimizes the angle and doesn't end
   * 
   * @param angle The angle between the target and the current heading of the robot
   * @param driveSubsystem The drive subsystem of the robot
  */
  public TurnToAngle(double angle, DriveSubsystem driveSubsystem) {
    this.angle = angle;
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
    if(m_DriveSubsystem.getHeading() < angle) {
      m_DriveSubsystem.drive(0, 0, -0.2, false, true);
    } else if(m_DriveSubsystem.getHeading() > angle) {
      m_DriveSubsystem.drive(0, 0, 0.2, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(angle - m_DriveSubsystem.getHeading()) <= 10) {
      return true;
    } else {
      return false;
    }
  }
}
