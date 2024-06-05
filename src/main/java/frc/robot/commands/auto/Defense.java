// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class Defense extends Command {

  private Supplier<Double> tx;
  private DriveSubsystem driveTrain;

  /** Creates a new Defense. */
  public Defense(Supplier<Double> tx, DriveSubsystem driveTrain) {
    this.tx = tx;
    this.driveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tx.get() > 0.1) {
      driveTrain.drive(0.5, 0, 0, true, true);
    } else if(tx.get() < -0.1) {
      driveTrain.drive(-0.5, 0, 0, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
