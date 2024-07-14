// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class VisionPivot extends Command {

  private AtomicReference<Double> ta;
  private Pivot m_Pivot;

  /** Creates a new VisionPivot. */
  public VisionPivot(AtomicReference<Double> ta, Pivot pivot) {
    this.ta = ta;
    m_Pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = shooterFunction(ta.get(),
        PivotConstants.kVisionA,
        PivotConstants.kVisionB,
        PivotConstants.kVisionC,
        PivotConstants.kVisionD);

    m_Pivot.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Pivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Variable rational function
   * @param x the input/independent variable
   * @param a
   * @param b
   * @param c
   * @param d
   * @return the Y value for the given X value
   */
  private double shooterFunction(double x, double a, double b, double c, double d) {
    return ((a*-x)+b)/((c*-x)+d);
  }
}
