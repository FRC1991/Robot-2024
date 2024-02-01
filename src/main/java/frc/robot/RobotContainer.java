// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain.BangBang.RunToTarget;
import frc.robot.commands.drivetrain.BangBang.TurnToTarget;
import frc.robot.commands.drivetrain.PID.TurnToAnglePID;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //region - network tables
  public static AtomicReference<Double> tv = new AtomicReference<Double>();
  public static AtomicReference<Double> tx = new AtomicReference<Double>();
  public static AtomicReference<Double> ty = new AtomicReference<Double>();
  public static AtomicReference<Double> tid = new AtomicReference<Double>();
  public static AtomicReference<Double> ta = new AtomicReference<Double>();

  private DoubleTopic dlbTopic_tv;
  private DoubleTopic dlbTopic_tx;
  private DoubleTopic dlbTopic_ty;
  private DoubleTopic dlbTopic_tid;
  private DoubleTopic dlbTopic_ta;


  public double tvHandle; //Whether the limelight has any valid targets (0 or 1)
  public double txHandle; //Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  public double tyHandle; //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  public double tidHandle; //ID of the primary in-view AprilTag
  public double taHandle; //Target Area (0% of image to 100% of image)

  public static AtomicReference<Double> intaketv = new AtomicReference<Double>();
  public static AtomicReference<Double> intaketx = new AtomicReference<Double>();
  public static AtomicReference<Double> intakety = new AtomicReference<Double>();
  public static AtomicReference<Double> intaketid = new AtomicReference<Double>();

  private DoubleTopic intakeDlbTopic_tv;
  private DoubleTopic intakeDlbTopic_tx;
  private DoubleTopic intakeDlbTopic_ty;
  private DoubleTopic intakeDlbTopic_tid;


  public double intakeTvHandle; //Whether the limelight has any valid targets (0 or 1)
  public double intakeTxHandle; //Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  public double intakeTyHandle; //Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  public double intakeTidHandle; //ID of the primary in-view AprilTag
  //endregion
  
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Intake m_Intake = new Intake();

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  public final XboxController m_auxController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureNetworkTables();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        //     () -> m_robotDrive.drive(
        //         -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
        //         true, false, DriveConstants.speedScale),
        //     m_robotDrive));

        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverJoytick.getRawAxis(1), 0.1),
                -MathUtil.applyDeadband(driverJoytick.getRawAxis(0), 0.1),
                -MathUtil.applyDeadband(driverJoytick.getRawAxis(2), 0.1),
                true, true, 0.8),
            m_robotDrive));

    m_Intake.setDefaultCommand(new RunIntake(m_auxController.getLeftY(), m_Intake));
    // m_Intake.setDefaultCommand(
    //     new RunCommand(() -> m_Intake.setIntakeSpeed(m_auxController.getLeftY()),
    //         m_Intake));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //Stops movement while Button.kR1.value is held down
    new JoystickButton(driverJoytick, 1)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    //Forms a tank drivetrain while Button.kL1.value is held down
    new JoystickButton(driverJoytick, 3)
        .whileTrue(new RunToTarget(() -> intaketx.get(), () -> 0.3, m_robotDrive));

    new JoystickButton(driverJoytick, 2)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(driverJoytick, 4)
        .onTrue(new TurnToTarget(() -> intaketx.get(), m_robotDrive));

    new JoystickButton(driverJoytick, 5)
        .whileTrue(new TurnToAnglePID(270, m_robotDrive));
    
    new JoystickButton(driverJoytick, 6)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.setSpeedScale(driverJoytick.getRawAxis(3))
          ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(2, 1), new Translation2d(4, -1)),
        // End 6 meters straight ahead of where we started, facing forward
        new Pose2d(6, 0, new Rotation2d(180)),
        config);
    
    Trajectory exampleTrajectoryPt2 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(6, 0, new Rotation2d(180)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(2, 1), new Translation2d(1, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0, new Rotation2d(0)),
        config);

    // When combining both 's' shaped trajectories, a figure 8 shaped trajectory is formed
    exampleTrajectory.concatenate(exampleTrajectoryPt2);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false, 0));
  }

  /**
   * Configures the network tables for the robot. 
   * Also adds listeners to automatically update the network table values.
   * 
   * Use the AtomicReference<Double> values when trying to reference
   * a value from a network table
   */
  public void configureNetworkTables() {
    NetworkTableInstance defaultNTinst = NetworkTableInstance.getDefault();
    //NetworkTable aimingLime = defaultNTinst.getTable("limelight-aimming");

    NetworkTable intakeLime = defaultNTinst.getTable("limelight-intake");

    // dlbTopic_tv = aimingLime.getDoubleTopic("tv");

    //  tvHandle = defaultNTinst.addListener(
    //   dlbTopic_tv,
    //   EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
    //   event -> {
    //     tv.set(event.valueData.value.getDouble());
    //   }
    //  );

    // dlbTopic_tx = aimingLime.getDoubleTopic("tx");

    //  txHandle = defaultNTinst.addListener(
    //   dlbTopic_tx,
    //   EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
    //   event -> {
    //     tx.set(event.valueData.value.getDouble());
    //   }
    //  );

    // dlbTopic_ty = aimingLime.getDoubleTopic("ty");

    //  tyHandle = defaultNTinst.addListener(
    //   dlbTopic_ty,
    //   EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
    //   event -> {
    //     ty.set(event.valueData.value.getDouble());
    //   }
    // );

    //  dlbTopic_tid = aimingLime.getDoubleTopic("tid");

    //  tidHandle = defaultNTinst.addListener(
    //   dlbTopic_tid,
    //   EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
    //   event -> {
    //     tid.set(event.valueData.value.getDouble());
    //   }
    //  );

    //  dlbTopic_ta = aimingLime.getDoubleTopic("a");

    //  aHandle = defaultNTinst.addListener(
    //   dlbTopic_ta,
    //   EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
    //   event -> {
    //     ta.set(event.valueData.value.getDouble());
    //   }
    //  );

    intakeDlbTopic_tv = intakeLime.getDoubleTopic("tv");

     intakeTvHandle = defaultNTinst.addListener(
      intakeDlbTopic_tv,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
      event -> {
        intaketv.set(event.valueData.value.getDouble());
      }
     );

    intakeDlbTopic_tx = intakeLime.getDoubleTopic("tx");

     intakeTxHandle = defaultNTinst.addListener(
      intakeDlbTopic_tx,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
      event -> {
        intaketx.set(event.valueData.value.getDouble());
      }
    );

    intakeDlbTopic_ty = intakeLime.getDoubleTopic("ty");

     intakeTyHandle = defaultNTinst.addListener(
      intakeDlbTopic_ty,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
      event -> {
        intakety.set(event.valueData.value.getDouble());
      }
     );

    intakeDlbTopic_tid = intakeLime.getDoubleTopic("tid");

     intakeTidHandle = defaultNTinst.addListener(
      intakeDlbTopic_tid,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
      event -> {
        intaketid.set(event.valueData.value.getDouble());
      }
     );
  }
}
