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
  DriveSubsystem m_DriveSubsystem;
  double targetAngle;
  /** Creates a new TurnToAnglePID. */
  public TurnToAnglePID(double targetAngle, DriveSubsystem driveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.9, 0, 0),
        // This should return the measurement
        () -> driveSubsystem.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> targetAngle,
        // This uses the output
        output -> {
          driveSubsystem.drive(0, 0, output, false, true);
        });
    
    this.targetAngle = targetAngle;
    m_DriveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
