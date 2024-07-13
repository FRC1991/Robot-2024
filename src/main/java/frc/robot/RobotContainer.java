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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.auto.Defense;
import frc.robot.commands.auto.Interference;
import frc.robot.commands.auto.PIDDefense;
import frc.robot.commands.drivetrain.PID.TurnToSource;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.PIDPivotToSetpoint;
import frc.robot.commands.pivot.PIDVisionPivot;
import frc.robot.commands.pivot.RunPivot;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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

  // The proximity sensor detecting the presence of a note in the Intake
  private final DigitalInput proximitySensor = new DigitalInput(0);
  private final Trigger proximityTrigger = new Trigger(proximitySensor::get);

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final SendableChooser<InstantCommand> angleChooser = new SendableChooser<InstantCommand>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configures network table listeners
    configureNetworkTables();

    // Configures wigets for the driver station
    configureShuffleBoard();

    // Configure default commands
    DriveSubsystem.getInstance().setDefaultCommand(
        // The joystick controls translation of the robot.
        // Turning is controlled by rotating the joystick.
        new RunCommand(
            () -> DriveSubsystem.getInstance().drive(
                -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OperatingInterface.driverJoytick.getRawAxis(2), OIConstants.kDriveDeadband),
                true, false, TeleopConstants.kSwerveSpeed),
            DriveSubsystem.getInstance()));

    // Intake.getInstance().setDefaultCommand(new RunIntake(OperatingInterface.auxController::getLeftY, Intake.getInstance()));

    // Shooter.getInstance().setDefaultCommand(new VisionShooter(ta, tid, Shooter.getInstance()));

    Pivot.getInstance().setDefaultCommand(new RunPivot(OperatingInterface.auxController::getLeftY, Pivot.getInstance()));

    // Configures the button bindings
    configureButtonBindings();

    // Configures the limelights for the match
    configureLimelights();
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
    new JoystickButton(OperatingInterface.driverJoytick, 1)
        .whileTrue(new RunCommand(
            () -> DriveSubsystem.getInstance().setX(),
            DriveSubsystem.getInstance()));

    // Zeros out the gyro (bottom thumb button)
    new JoystickButton(OperatingInterface.driverJoytick, 2)
        .onTrue(new InstantCommand(
            () -> DriveSubsystem.getInstance().zeroHeading(),
            DriveSubsystem.getInstance()));

    new JoystickButton(OperatingInterface.driverJoytick, 5)
        .onTrue(new InstantCommand(
            () -> Pivot.getInstance().zeroMotorEncoders(),
            Pivot.getInstance()));

    new JoystickButton(OperatingInterface.driverJoytick, 8)
        .whileTrue(new TurnToSource(145, DriveSubsystem.getInstance()));

    // new JoystickButton(OperatingInterface.driverJoytick, 9)
    //     .whileTrue(new PIDTurnToTarget(intaketx::get, OperatingInterface, DriveSubsystem.getInstance()));

    // new JoystickButton(OperatingInterface.driverJoytick, 14)
    //     .whileTrue(new PIDTurnToTarget(tx::get, OperatingInterface, DriveSubsystem.getInstance()));

    // OperatingInterface.auxRightBumper.whileTrue(new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         new RunShooter(() -> 1.0, Shooter.getInstance()),
    //         new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance())).withTimeout(0.8),
    //     new ParallelCommandGroup(
    //         new RunShooter(() -> 1.0, Shooter.getInstance()),
    //         new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance()),
    //         new RunIntake(() -> 0.8, Intake.getInstance()))));

    OperatingInterface.auxYButton.whileTrue(new PIDVisionPivot(() -> ty.get(), () -> 0.0, Pivot.getInstance()));

    // Shooting
    OperatingInterface.auxRightBumper.whileTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
            new RunShooter(() -> 1.0, Shooter.getInstance()),
            new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance())).withTimeout(0.8),
        new ParallelCommandGroup(
            new RunShooter(() -> 1.0, Shooter.getInstance()),
            new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance()),
            new RunIntake(() -> 0.8, Intake.getInstance()))));

    OperatingInterface.auxStartButton.whileTrue(new SequentialCommandGroup(
        new ParallelCommandGroup(
            new RunShooter(() -> 1.0, Shooter.getInstance()),
            new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kLowShotPosition, Pivot.getInstance())).withTimeout(0.6),
        new ParallelCommandGroup(
            new RunShooter(() -> 1.0, Shooter.getInstance()),
            new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kLowShotPosition, Pivot.getInstance()),
            new RunIntake(() -> 0.8, Intake.getInstance()))));
    // OperatingInterface.auxRightBumper.whileTrue(getOnePieceAuto());

    OperatingInterface.auxBButton.whileTrue(new RunIntake(() -> 0.8, Intake.getInstance()));
    // OperatingInterface.auxXButton.onTrue(new RunIntake(() -> 0.0, Intake.getInstance()));
    OperatingInterface.auxXButton.whileTrue(new RunShooter(() -> 1.0, Shooter.getInstance()));
    proximityTrigger.onTrue(new InstantCommand(OperatingInterface::rumbleAuxController));
    OperatingInterface.auxAButton.whileTrue(new RunIntake(() -> -0.6, Intake.getInstance()));

    OperatingInterface.auxLeftBumper.whileTrue(new PIDPivotToSetpoint(() -> 0.1, () -> 0.0, Pivot.getInstance()));
    // OperatingInterface.auxXButton.whileTrue(new PIDPivotToSetpoint(() -> 0.1, () -> -AutoConstants.kSpeakerEncoderPosition, Pivot.getInstance()));
  }

  public void configureShuffleBoard() {
    // For easy configuration of the gyro at the start of every match
    angleChooser.addOption("0", new InstantCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(0), DriveSubsystem.getInstance()));
    angleChooser.addOption("180", new InstantCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(180), DriveSubsystem.getInstance()));
    angleChooser.addOption("120", new InstantCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(120), DriveSubsystem.getInstance()));
    angleChooser.addOption("240", new InstantCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(240), DriveSubsystem.getInstance()));
    angleChooser.addOption("330", new InstantCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(330), DriveSubsystem.getInstance()));
    angleChooser.addOption("30", new InstantCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(30), DriveSubsystem.getInstance()));

    // To use in the Path Planner GUI
    NamedCommands.registerCommand("Run Shooter", new RunShooter(() -> 1.0, Shooter.getInstance()));
    NamedCommands.registerCommand("Pivot to Setpoint", new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance()));
    NamedCommands.registerCommand("Pivot flat", new PIDPivotToSetpoint(() -> 0.1, () -> 0.0, Pivot.getInstance()));
    NamedCommands.registerCommand("Run Intake", new RunIntake(() -> 0.8, Intake.getInstance()));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> Intake.getInstance().setIntakeSpeed(0), Intake.getInstance()));
    NamedCommands.registerCommand("Run Intake - proximity sensor", new RunIntake(() -> 0.8, Intake.getInstance()).onlyWhile(proximityTrigger));
    NamedCommands.registerCommand("Stop drivetrain", new RunCommand(() -> DriveSubsystem.getInstance().drive(0,0,0,false,false,0), DriveSubsystem.getInstance()));
    // NamedCommands.registerCommand("gyro to 240", new RunCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(240), DriveSubsystem.getInstance()));
    // NamedCommands.registerCommand("gyro to 180", new RunCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(180), DriveSubsystem.getInstance()));
    // NamedCommands.registerCommand("gyro to 120", new RunCommand(() -> DriveSubsystem.getInstance().m_gyro.setYaw(120), DriveSubsystem.getInstance()));

    // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    // All from PathPlanner and don't work
    autoChooser.addOption("Blue Midside One note + movement", new PathPlannerAuto("mid One Note Blue"));
    autoChooser.addOption("Blue Openside One note + movement", new PathPlannerAuto("open One Note Blue"));
    autoChooser.addOption("Blue Ampside One note + movement", new PathPlannerAuto("amp One Note Blue"));
    autoChooser.addOption("Red Openside One note + movement", new PathPlannerAuto("open One Note Red"));
    autoChooser.addOption("Red Midside One note + movement", new PathPlannerAuto("mid One Note Red"));
    autoChooser.addOption("Red Ampside One note + movement", new PathPlannerAuto("amp One Note Red"));
    autoChooser.addOption("Interference Blue", new PathPlannerAuto("InterferenceAutoBlue"));
    autoChooser.addOption("Interference Red", new PathPlannerAuto("InterferenceAutoRed"));
    autoChooser.addOption("Two Note Blue", new PathPlannerAuto("Two Note Blue"));
    autoChooser.addOption("Two Note Red", new PathPlannerAuto("Two Note Red"));

    // Untested
    autoChooser.addOption("BangBang Defense", new SequentialCommandGroup(
          new RunCommand(
            () -> DriveSubsystem.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            DriveSubsystem.getInstance()).withTimeout(2),
          new Defense(() -> tx.get(), DriveSubsystem.getInstance())
        ));
    autoChooser.addOption("PID Defense", new SequentialCommandGroup(
          new RunCommand(
            () -> DriveSubsystem.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            DriveSubsystem.getInstance()).withTimeout(2),
          new PIDDefense(() -> tx.get(), DriveSubsystem.getInstance())
        ));

    // Hard coded autos
    autoChooser.addOption("Nothing", new InstantCommand());
    autoChooser.addOption("One note mid", getOnePieceAuto());
    autoChooser.addOption("One note side", getOnePieceAuto());
    autoChooser.addOption("One note mid + movement", getOnePieceAutoMovementCommand());
    autoChooser.addOption("Two note + movement", getTwoPieceAutoCommand());
    autoChooser.addOption("manual blue interference auto", new SequentialCommandGroup(
        new RunCommand(
            () -> DriveSubsystem.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            DriveSubsystem.getInstance()).withTimeout(2),
        new Interference(true, DriveSubsystem.getInstance()).withTimeout(1.9)
    ));
    autoChooser.addOption("manual red interference auto", new SequentialCommandGroup(
        new RunCommand(
            () -> DriveSubsystem.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            DriveSubsystem.getInstance()).withTimeout(2),
        new Interference(false, DriveSubsystem.getInstance()).withTimeout(1.9)
    ));

    // Booleans
    // Shuffleboard.getTab("Main").addBoolean("intaking?", () -> OperatingInterface.auxBButton.getAsBoolean());
    Shuffleboard.getTab("Main").addBoolean("proximity sensor", proximitySensor::get);
    // Shuffleboard.getTab("Main").addBoolean("lower limit switch", lowerPivotLimit::get);
    // Shuffleboard.getTab("Main").addBoolean("upper limit switch", upperPivotLimit::get);

    // // Doubles
    Shuffleboard.getTab("Main").addDouble("Heading", DriveSubsystem.getInstance()::getHeading);

    Shuffleboard.getTab("Main").addDouble("pivot encoder", Pivot.getInstance()::getEncoderPosition);
    // Shuffleboard.getTab("Network Table Values").addDouble("shooter ta", ta::get);
    // Shuffleboard.getTab("Network Table Values").addDouble("shooter tid", tid::get);
    // Shuffleboard.getTab("Network Table Values").addDouble("shooter thor", thor::get);
    // Shuffleboard.getTab("Network Table Values").addDouble("shooter tvert", tvert::get);
    // Shuffleboard.getTab("Network Table Values").addDouble("intake ta", intaketa::get);
    // Shuffleboard.getTab("Network Table Values").addDouble("intake tid", intaketid::get);
    // Shuffleboard.getTab("Network Table Values").addDouble("intake tx", intaketx::get);
    Shuffleboard.getTab("Main").add("auto chooser", autoChooser);
    Shuffleboard.getTab("Main").add("angle chooser", angleChooser);
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
   *
   * @return A new ParallelRaceGroup which shoots the preloaded note and does not move
   */
  public ParallelRaceGroup getOnePieceAuto() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new RunShooter(() -> 1.0, Shooter.getInstance()),
            new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance())).withTimeout(0.8),
        new ParallelCommandGroup(
            new RunShooter(() -> 1.0, Shooter.getInstance()),
            new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance()),
            new RunIntake(() -> 0.8, Intake.getInstance()))).withTimeout(7);

  }

  /**
   *
   * @return A one piece auto with a run command which moves the robot at the end
   */
  public Command getOnePieceAutoMovementCommand() {
    ParallelRaceGroup auto = getOnePieceAuto();

    auto.addCommands(
              new RunCommand(() -> {}, DriveSubsystem.getInstance()).withTimeout(10),
              new RunCommand(
            () -> DriveSubsystem.getInstance().drive(0.3, 0, 0,
                true, false, TeleopConstants.kSwerveSpeed),
            DriveSubsystem.getInstance()).withTimeout(2));

    return auto;
  }

  /**
   * DON'T USE UNTIL FURTHER TESTING HAS BEEN CONDUCTED
   * @return
   */
  public Command getTwoPieceAutoCommand() {
    SequentialCommandGroup auto = new SequentialCommandGroup();

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // X AND Y MIGHT BE SWITCHED
    // // An example trajectory to follow. All units in meters.
    // Trajectory pickUpNote = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1,0)),
    //     // End 6 meters straight ahead of where we started, facing forward
    //     new Pose2d(2, 0, new Rotation2d(180)),
    //     config);

    // // X AND Y MIGHT BE SWITCHED
    // Trajectory fromNoteToSpeaker = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(2, 0, new Rotation2d(180)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 0)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand pickUpNoteCommand = new SwerveControllerCommand(
    //     pickUpNote,
    //     DriveSubsystem.getInstance()::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     DriveSubsystem.getInstance()::setModuleStates,
    //     DriveSubsystem.getInstance());

    // SwerveControllerCommand toSpeakerCommand = new SwerveControllerCommand(
    //     fromNoteToSpeaker,
    //     DriveSubsystem.getInstance()::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     DriveSubsystem.getInstance()::setModuleStates,
    //     DriveSubsystem.getInstance());

    auto.addCommands(getOnePieceAuto(),
        new ParallelCommandGroup(
          new RunIntake(() -> 0.8, Intake.getInstance()),
          new RunCommand(
            () -> DriveSubsystem.getInstance().drive(-0.3, 0, 0,
                false, false, TeleopConstants.kSwerveSpeed),
            DriveSubsystem.getInstance())
        ).withTimeout(1.5),

        new ParallelCommandGroup(
          new RunIntake(() -> -0.1, Intake.getInstance()).withTimeout(0.5),
          new RunCommand(
              () -> DriveSubsystem.getInstance().drive(0.3, 0, 0,
                  false, false, TeleopConstants.kSwerveSpeed),
              DriveSubsystem.getInstance()).withTimeout(1.5)
        ),
        getOnePieceAuto());

    // Reset odometry to the starting pose of the trajectory.
    // DriveSubsystem.getInstance().resetOdometry(pickUpNote.getInitialPose());

    // Run path following command, then stop at the end.
    return auto.andThen(() -> DriveSubsystem.getInstance().drive(0, 0, 0, false, false, 0));
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
        DriveSubsystem.getInstance()::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        DriveSubsystem.getInstance()::setModuleStates,
        DriveSubsystem.getInstance());

    // Reset odometry to the starting pose of the trajectory.
    DriveSubsystem.getInstance().resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> DriveSubsystem.getInstance().drive(0, 0, 0, false, false, 0));
  }

  public void configureGyro() {
    angleChooser.getSelected().schedule();
  }

  /**
   * Make sure to tune your pipelines at every event. This is to ensure you get accurate readings
   * under varying lighting conditions.
   *
   * Automatically configures the intake limelight to pipeline zero.
   *
   * Automatically configures the shooter limelight to pipeline zero if
   * on the blue alliance and pipeline one if on the red alliance.
   */
  public void configureLimelights() {
    if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      LimelightHelpers.setPipelineIndex("limelight-shooter", 0);
    } else {
      LimelightHelpers.setPipelineIndex("limelight-shooter", 1);
    }

    LimelightHelpers.setPipelineIndex("limelight-intake", 0);
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
    NetworkTable aimingLime = defaultNTinst.getTable("limelight-shooter");
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
