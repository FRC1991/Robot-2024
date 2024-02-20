// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class VisionShooter extends Command {

  private AtomicReference<Double> ta, tid;
  private Shooter m_Shooter;

  /** Creates a new VisionShooter. */
  public VisionShooter(AtomicReference<Double> ta, AtomicReference<Double> tid, Shooter shooter) {

    this.ta = ta;
    this.tid = tid;
    m_Shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If one of the Speaker's april tags is in view, set Shooter to the size of the april tag
    if(tid.get() == 1 || tid.get() ==2) {
      m_Shooter.setShooter(ta.get() * ShooterConstants.kVisionCoefficient);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
