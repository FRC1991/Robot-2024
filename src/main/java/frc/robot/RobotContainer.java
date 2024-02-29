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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.drivetrain.BangBang.RunToTarget;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.PIDPivotToSetpoint;
import frc.robot.commands.pivot.PivotToSetpoint;
import frc.robot.commands.pivot.RunPivot;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.VisionShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
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
  public static AtomicReference<Double> thor = new AtomicReference<Double>();
  public static AtomicReference<Double> tvert = new AtomicReference<Double>();

  private DoubleTopic dlbTopic_tv;
  private DoubleTopic dlbTopic_tx;
  private DoubleTopic dlbTopic_ty;
  private DoubleTopic dlbTopic_tid;
  private DoubleTopic dlbTopic_ta;
  private DoubleTopic dlbTopic_thor;
  private DoubleTopic dlbTopic_tvert;


  public double tvHandle;// Whether the limelight has any valid targets (0 or 1)
  public double txHandle;// Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  public double tyHandle;// Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  public double tidHandle;// ID of the primary in-view AprilTag
  public double taHandle;// Target Area (0% of image to 100% of image)
  public double thorHandle;// Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  public double tvertHandle;// Vertical sidelength of the rough bounding box (0 - 320 pixels)

  public static AtomicReference<Double> intaketv = new AtomicReference<Double>();
  public static AtomicReference<Double> intaketx = new AtomicReference<Double>();
  public static AtomicReference<Double> intakety = new AtomicReference<Double>();
  public static AtomicReference<Double> intaketid = new AtomicReference<Double>();
  public static AtomicReference<Double> intaketa = new AtomicReference<Double>();

  private DoubleTopic intakeDlbTopic_tv;
  private DoubleTopic intakeDlbTopic_tx;
  private DoubleTopic intakeDlbTopic_ty;
  private DoubleTopic intakeDlbTopic_tid;
  private DoubleTopic intakeDlbTopic_ta;


  public double intakeTvHandle;// Whether the limelight has any valid targets (0 or 1)
  public double intakeTxHandle;// Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  public double intakeTyHandle;// Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  public double intakeTidHandle;// ID of the primary in-view AprilTag
  public double intakeTaHandle;// Target Area (0% of image to 100% of image)
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
  private final DigitalInput proximity = new DigitalInput(9);

  // Limit switches stopping the Pivot from moving too far
  //TODO are these even going to be used?
  private final DigitalInput upperPivotLimit = new DigitalInput(1);
  private final DigitalInput lowerPivotLimit = new DigitalInput(2);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private GenericEntry intakeSpeed;
  private GenericEntry shooterSpeed;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configures network table listeners
    configureNetworkTables();

    NamedCommands.registerCommand("Run Shooter", new RunShooter(() -> TeleopConstants.kShooterSpeed, m_Shooter));
    //autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`

    // Configures wigets for the driver station
    configureShuffleBoard();

    // Configure default commands
    m_DriveTrain.setDefaultCommand(
        // The joystick controls translation of the robot.
        // Turning is controlled by rotating the joystick.
        new RunCommand(
            () -> m_DriveTrain.drive(
                -MathUtil.applyDeadband(oi.driverJoytick.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(oi.driverJoytick.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(oi.driverJoytick.getRawAxis(2), OIConstants.kDriveDeadband),
                true, false, TeleopConstants.kSwerveSpeed),
            m_DriveTrain));

    // m_Intake.setDefaultCommand(new RunIntake(oi.auxController::getLeftY, m_Intake));

    // m_Shooter.setDefaultCommand(new VisionShooter(ta, tid, m_Shooter));

    // m_Pivot.setDefaultCommand(new RunPivot(oi.auxController::getLeftY, m_Pivot));

    // Configures the button bindings
    configureButtonBindings();
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

    // Stops movement while Button.kR1.value is held down
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

    new JoystickButton(oi.driverJoytick, 5)
        .whileTrue(new RunToTarget(() -> tx.get(), () -> 0.1, m_DriveTrain));

    oi.auxRightBumper.whileTrue(new RunShooter(() -> shooterSpeed.get().getDouble(), m_Shooter));

    // oi.auxXButton.whileTrue(new RunClimber(() -> TeleopConstants.kClimberSpeed, m_Climber));

    oi.auxBButton.whileTrue(new RunIntake(() -> intakeSpeed.get().getDouble(), m_Intake));
    oi.auxAButton.whileTrue(new RunIntake(() -> -intakeSpeed.get().getDouble(), m_Intake));
    // oi.auxBButton.whileTrue(new RunCommand(() -> System.out.println("b button pressed")));

    oi.auxLeftBumper.whileTrue(new PIDPivotToSetpoint(() -> 0.1, () -> -5.2, m_Pivot));
  }

  public void configureShuffleBoard() {
    // Booleans
    Shuffleboard.getTab("Main").addBoolean("intaking?", () -> oi.auxBButton.getAsBoolean());
    Shuffleboard.getTab("Main").addBoolean("proximity sensor", proximity::get);
    // Shuffleboard.getTab("Main").addBoolean("lower limit switch", lowerPivotLimit::get);
    // Shuffleboard.getTab("Main").addBoolean("upper limit switch", upperPivotLimit::get);

    // // Doubles
    Shuffleboard.getTab("Main").addDouble("Heading", m_DriveTrain::getHeading);
    intakeSpeed = Shuffleboard.getTab("Main").add("Intake speed", 0.8)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();

    shooterSpeed = Shuffleboard.getTab("Main").add("Shooter speed", 0.8)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();

    Shuffleboard.getTab("Main").addDouble("pivot encoder", m_Pivot::getEncoderPosition);
    // Shuffleboard.getTab("Main").addDouble("shooter ta", ta::get);
    // Shuffleboard.getTab("Main").addDouble("shooter tid", tid::get);
    // Shuffleboard.getTab("Main").addDouble("shooter thor", thor::get);
    // Shuffleboard.getTab("Main").addDouble("shooter tvert", tvert::get);
    // Shuffleboard.getTab("Main").addDouble("intake ta", intaketa::get);
    // Shuffleboard.getTab("Main").addDouble("intake tid", intaketid::get);

    // Shuffleboard.getTab("Main").add(autoChooser);
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

     dlbTopic_ta = aimingLime.getDoubleTopic("ta");

     taHandle = defaultNTinst.addListener(
      dlbTopic_ta,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        ta.set(event.valueData.value.getDouble());
      }
     );

     dlbTopic_thor = aimingLime.getDoubleTopic("thor");

     thorHandle = defaultNTinst.addListener(
      dlbTopic_thor,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        thor.set(event.valueData.value.getDouble());
      }
     );

     dlbTopic_tvert = aimingLime.getDoubleTopic("tvert");

     tvertHandle = defaultNTinst.addListener(
      dlbTopic_tvert,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        tvert.set(event.valueData.value.getDouble());
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

     intakeDlbTopic_ta = intakeLime.getDoubleTopic("ta");

     intakeTaHandle = defaultNTinst.addListener(
      intakeDlbTopic_ta,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        intaketa.set(event.valueData.value.getDouble());
      }
     );
  }
}
