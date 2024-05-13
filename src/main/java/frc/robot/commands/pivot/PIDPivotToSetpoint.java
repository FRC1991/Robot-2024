// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDPivotToSetpoint extends PIDCommand {
  /** Creates a new PIDPivottoSetpoint. */
  public PIDPivotToSetpoint(Supplier<Double> kp, Supplier<Double> setpoint, Pivot pivot) {
    super(
        // The controller that the command will use
        new PIDController(0.1, 0, 0),
        // This should return the measurement
        pivot::getEncoderPosition,
        // This should return the setpoint (can also be a constant)
        setpoint::get,
        // This uses the output
        output -> {
          // Use the output here
          pivot.setPivot(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
