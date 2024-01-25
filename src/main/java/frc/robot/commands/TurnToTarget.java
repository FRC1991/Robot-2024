// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTarget extends Command {

  private Supplier<Double> xDiff;
  private DriveSubsystem m_DriveSubsystem;
  /** 
   * Creates a command that minimizes the xDiff and doesn't end
   * 
   * @param xDiff The angle between the target and the current heading of the robot
   * @param driveSubsystem The drive subsystem of the robot
  */
  public TurnToTarget(Supplier<Double> xDiff, DriveSubsystem driveSubsystem) {
    this.xDiff = xDiff;
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
    if(xDiff.get() > 2) {
      m_DriveSubsystem.drive(0, 0, 0.2, false, true);
    } else if(xDiff.get() < -2) {
      m_DriveSubsystem.drive(0, 0, -0.2, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(xDiff.get()) <= 10) {
      return true;
    } else {
      return false;
    }
  }
}
