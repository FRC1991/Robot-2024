// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends Command {

  private Supplier<Double> angle;
  private DriveSubsystem m_DriveSubsystem;
  /** 
   * Creates a command that minimizes the xDiff and doesn't end
   * 
   * @param angle The desired angle to arrive at
   * @param driveSubsystem The drive subsystem of the robot
  */
  public TurnToAngle(Supplier<Double> angle, DriveSubsystem driveSubsystem) {
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
    if(angle.get() > 180) {
      m_DriveSubsystem.drive(0, 0, 1, false, true, 0.2);
    } else if(angle.get() <= 180) {
      m_DriveSubsystem.drive(0, 0, -1, false, true, 0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(angle.get()) < 0.05) {
      return true;
    } else {
      return false;
    }
  }
}
