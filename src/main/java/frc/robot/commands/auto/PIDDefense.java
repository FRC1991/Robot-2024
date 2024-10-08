// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDDefense extends PIDCommand {
  /** Creates a new PIDDefense. */
  public PIDDefense(Supplier<Double> tx, Swerve driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(0.009, 0, 0),
        // This should return the measurement
        tx::get,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          driveTrain.drive(output, 0, 0, true, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
