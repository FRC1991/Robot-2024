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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.RunPivot;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.VisionShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final DriveSubsystem m_DriveTrain = new DriveSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();
  private final Pivot m_Pivot = new Pivot();
  private final Climber m_Climber = new Climber();

  // The operating interface communicating with the user
  private final OperatingInterface oi = new OperatingInterface();

  // The proximity sensor detecting the presence of a note in the Intake
  private final DigitalInput proximity = new DigitalInput(0);

  // Limit switches stopping the Pivot from moving too far
  //TODO are these even going to be used?
  private final DigitalInput upperPivotLimit = new DigitalInput(1);
  private final DigitalInput lowerPivotLimit = new DigitalInput(2);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configures the button bindings
    configureButtonBindings();
    // Configures network table listeners
    configureNetworkTables();
    // Configures wigets for the driver station
    configureShuffleBoard();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    NamedCommands.registerCommand("RunShooter", new RunShooter(() -> 0.8, m_Shooter));

    // Configure default commands
    m_DriveTrain.setDefaultCommand(
        // The joystick controls translation of the robot.
        // Turning is controlled by rotating the joystick.
        new RunCommand(
            () -> m_DriveTrain.drive(
                -MathUtil.applyDeadband(oi.driverJoytick.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(oi.driverJoytick.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(oi.driverJoytick.getRawAxis(2), OIConstants.kDriveDeadband),
                true, false, 0.8),
            m_DriveTrain));

    m_Intake.setDefaultCommand(new RunIntake(oi.auxController.getLeftY(), m_Intake));

    m_Shooter.setDefaultCommand(new VisionShooter(ta, tid, m_Shooter));

    m_Pivot.setDefaultCommand(new RunPivot(() -> oi.auxController.getRightY(), m_Pivot));
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
    // Stops movement while Button.kR1.value (the trigger) is held down
    new JoystickButton(oi.driverJoytick, 1)
        .whileTrue(new RunCommand(
            () -> m_DriveTrain.setX(),
            m_DriveTrain));

    // Zeros out the gyro
    //TODO remove before going to competition 
    new JoystickButton(oi.driverJoytick, 2)
        .whileTrue(new RunCommand(
            () -> m_DriveTrain.zeroHeading(),
            m_DriveTrain));

    // oi.auxAButton.whileTrue(new RunShooter(() -> 0.3, m_Shooter));

    oi.auxXButton.whileTrue(new RunClimber(() -> 0.6, m_Climber));
  }

  public void configureShuffleBoard() {
    // Booleans
    Shuffleboard.getTab("Main").addBoolean("shooting?", () -> oi.auxController.getRawButton(1));
    Shuffleboard.getTab("Main").addBoolean("proximity sensor", proximity::get);

    // Doubles
    Shuffleboard.getTab("Main").addDouble("angle", m_DriveTrain::getHeading);

    Shuffleboard.getTab("Main").add(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * A great example of how to manually create autonomous commands.
   * 
   * @return the figure eight command
   */
  public Command getFigureEightCommand() {
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
        m_DriveTrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_DriveTrain::setModuleStates,
        m_DriveTrain);

    // Reset odometry to the starting pose of the trajectory.
    m_DriveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_DriveTrain.drive(0, 0, 0, false, false, 0));
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
    NetworkTable aimingLime = defaultNTinst.getTable("limelight-aimming");

    NetworkTable intakeLime = defaultNTinst.getTable("limelight-intake");

    dlbTopic_tv = aimingLime.getDoubleTopic("tv");

     tvHandle = defaultNTinst.addListener(
      dlbTopic_tv,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        tv.set(event.valueData.value.getDouble());
      }
     );

    dlbTopic_tx = aimingLime.getDoubleTopic("tx");

     txHandle = defaultNTinst.addListener(
      dlbTopic_tx,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        tx.set(event.valueData.value.getDouble());
      }
     );

    dlbTopic_ty = aimingLime.getDoubleTopic("ty");

     tyHandle = defaultNTinst.addListener(
      dlbTopic_ty,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        ty.set(event.valueData.value.getDouble());
      }
    );

     dlbTopic_tid = aimingLime.getDoubleTopic("tid");

     tidHandle = defaultNTinst.addListener(
      dlbTopic_tid,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        tid.set(event.valueData.value.getDouble());
      }
     );

     dlbTopic_ta = aimingLime.getDoubleTopic("a");

     taHandle = defaultNTinst.addListener(
      dlbTopic_ta,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        ta.set(event.valueData.value.getDouble());
      }
     );

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
