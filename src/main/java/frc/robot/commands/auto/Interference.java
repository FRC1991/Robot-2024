// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class Interference extends Command {

  private boolean blueSide;
  private DriveSubsystem driveTrain;

  /** Creates a new Interference. */
  public Interference(DriveSubsystem driveTrain) {
    this.driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blueSide = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(blueSide) {
      driveTrain.drive(0, 0.3, 0, true, false);
    } else {
      driveTrain.drive(0, -0.3, 0, true, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
