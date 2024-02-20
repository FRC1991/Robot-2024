// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotToSetpoint extends Command {

  private Supplier<Double> setpoint;
  private double tolerance;
  private Pivot m_Pivot;

  /** Creates a new PivotToAngle. */
  public PivotToSetpoint(Supplier<Double> setpoint, double tolerance, Pivot pivot) {
    this.setpoint = setpoint;
    this.tolerance = tolerance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    m_Pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Pivot.getAbsoluteEncoderValue() < (setpoint.get() - tolerance)) {
      m_Pivot.setPivot(0.4);
    } else if(m_Pivot.getAbsoluteEncoderValue() > (setpoint.get() + tolerance)) {
      m_Pivot.setPivot(-0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(setpoint.get() - m_Pivot.getAbsoluteEncoderValue()) <= tolerance) {
      return true;
    } else {
      return false;
    }
  }
}
