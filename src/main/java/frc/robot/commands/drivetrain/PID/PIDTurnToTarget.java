// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.PID;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.OperatingInterface;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTurnToTarget extends PIDCommand {
  /** Creates a new PIDTurnToTarget. */
  public PIDTurnToTarget(Supplier<Double> xDiff, DriveSubsystem driveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.009, 0, 0),
        // This should return the measurement
        () -> {
          try {
            return xDiff.get();
          } catch (Exception e) {
            return 0;
          }
        },
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          if(output > 1) {
            output = 1;
          } else if(output < -1) {
            output = -1;
          } else if (Math.abs(output) < .01) {
            output = 0;
          }
          driveSubsystem.drive(
              -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(1), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(0), OIConstants.kDriveDeadband),
              output, true, false, .6);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
