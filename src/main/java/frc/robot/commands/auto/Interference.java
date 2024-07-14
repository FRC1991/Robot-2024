// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Interference extends Command {

  private boolean blueSide;
  private Swerve driveTrain;

  /** Creates a new Interference. */
  public Interference(boolean blueSide, Swerve driveTrain) {
    this.driveTrain = driveTrain;
    this.blueSide = blueSide;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Goes the opposite direction depending on what alliance we are
    if(blueSide) {
      driveTrain.drive(0, 0.76257, -0.7, true, false);
    } else {
      driveTrain.drive(0, -0.76257, -0.7, true, false);
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
