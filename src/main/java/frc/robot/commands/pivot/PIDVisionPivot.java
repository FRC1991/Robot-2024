// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDVisionPivot extends PIDCommand {
  /** Creates a new PIDPivottoSetpoint. */
  public PIDVisionPivot(Supplier<Double> kp, Supplier<Double> ty, Pivot pivot) {
    super(
        // The controller that the command will use
        new PIDController(kp.get(), 0, 0),
        // This should return the measurement
        pivot::getEncoderPosition,
        // This should return the setpoint (can also be a constant)
        ty::get,
        // This uses the output
        output -> {
          double setpoint = ((PivotConstants.kVisionA * -output) + PivotConstants.kVisionB) / ((PivotConstants.kVisionC * -output) + PivotConstants.kVisionD);
          // Use the output here
          pivot.setPivot(setpoint);
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
