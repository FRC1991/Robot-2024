// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.BangBang;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunToTarget extends ParallelCommandGroup {
  /** Creates a new RunToTarget. */
  public RunToTarget(Supplier<Double> targetOffset, double speed, DriveSubsystem driveSubsystem) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new TurnToTarget(targetOffset, driveSubsystem) 
        /*new RunCommand(() -> driveSubsystem.drive(0, speed, 0, false, true), driveSubsystem)*/);
    // addCommands(new FooCommand(), new BarCommand());
  }
}
