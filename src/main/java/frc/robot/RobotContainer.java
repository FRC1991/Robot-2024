// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.auto.Defense;
import frc.robot.commands.auto.Interference;
import frc.robot.commands.auto.PIDDefense;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pivot.PIDPivotToSetpoint;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Manager.ManagerStates;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manager;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.EnumSet;
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

  private final Manager m_Manager = new Manager(tx::get, ty::get);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configures the limelights for the match
    configureLimelights();

    // Configures network table listeners
    configureNetworkTables();

    // Configures wigets for the driver station
    configureShuffleBoard();

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

    // Stops movement
    new JoystickButton(OperatingInterface.driverJoytick, 1)
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DEFENSE), m_Manager))
        .onFalse(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.DRIVE), m_Manager));

    // Zeros out the gyro (bottom thumb button)
    new JoystickButton(OperatingInterface.driverJoytick, 2)
        .onTrue(new InstantCommand(
            () -> Swerve.getInstance().zeroHeading(),
            Swerve.getInstance()));

    // Zeroing Pivot encoders
    new JoystickButton(OperatingInterface.driverJoytick, 5)
        .onTrue(new InstantCommand(
            () -> Pivot.getInstance().zeroMotorEncoders(),
            Pivot.getInstance()));

    // Note pick up
    new JoystickButton(OperatingInterface.driverJoytick, 8)
        .onTrue(new InstantCommand(() -> {
            if(intaketv.get() == 1) {
              m_Manager.setDesiredState(ManagerStates.SOURCE);
            } else {
              m_Manager.setDesiredState(ManagerStates.INTAKING);
            }
        }, m_Manager));

    // Shooting
    new JoystickButton(OperatingInterface.driverJoytick, 7)
        .onTrue(new InstantCommand(() -> {
            if(tv.get() == 1) {
              m_Manager.setDesiredState(ManagerStates.AIMMING);
            } else {
              m_Manager.setDesiredState(ManagerStates.SUBWOOFER_AIMMING);
            }
        }, m_Manager));

    // Outaking
    new JoystickButton(OperatingInterface.driverJoytick, 6)
        .onTrue(new InstantCommand(() -> m_Manager.setDesiredState(ManagerStates.OUTTAKING), m_Manager));
  }

  public void configureShuffleBoard() {
    ShuffleboardTab diagnostics = Shuffleboard.getTab("Diagnostics");
    ShuffleboardTab main = Shuffleboard.getTab("Main");

    // For easy configuration of the gyro at the start of every match
    angleChooser.addOption("0", new InstantCommand(() -> Swerve.getInstance().m_gyro.setYaw(0), Swerve.getInstance()));
    angleChooser.addOption("180", new InstantCommand(() -> Swerve.getInstance().m_gyro.setYaw(180), Swerve.getInstance()));
    angleChooser.addOption("120", new InstantCommand(() -> Swerve.getInstance().m_gyro.setYaw(120), Swerve.getInstance()));
    angleChooser.addOption("240", new InstantCommand(() -> Swerve.getInstance().m_gyro.setYaw(240), Swerve.getInstance()));
    angleChooser.addOption("330", new InstantCommand(() -> Swerve.getInstance().m_gyro.setYaw(330), Swerve.getInstance()));
    angleChooser.addOption("30", new InstantCommand(() -> Swerve.getInstance().m_gyro.setYaw(30), Swerve.getInstance()));
    main.add("auto chooser", autoChooser);

    // Adds the SendableChooser with all of the autos on it
    configureAutos();

    // The proximity sensor on the intake
    main.addBoolean("proximity sensor", proximitySensor::get);

    // Is the pivot ready to shoot
    main.addBoolean("Pivot at setpoint", Pivot.getInstance()::atSetpoint);

    // Robot heading
    diagnostics.addDouble("Heading", Swerve.getInstance()::getHeading);

    // Pivot angle
    diagnostics.addDouble("Pivot angle", Pivot.getInstance()::getEncoderPosition);

    // Limelight values
    diagnostics.addDouble("shooter tx", tx::get);
    diagnostics.addDouble("shooter ty", ty::get);
    diagnostics.addDouble("shooter tid", tid::get);
    diagnostics.addDouble("shooter tv (has target)", tv::get);
    diagnostics.addDouble("intake ta", intaketa::get);
    diagnostics.addDouble("intake tid", intaketid::get);
    diagnostics.addDouble("intake tx", intaketx::get);
    diagnostics.addDouble("intake tv (has target)", intaketv::get);


    // Initialization status
    diagnostics.addBoolean("Manager initialized", m_Manager::getInitialized);
    diagnostics.addBoolean("Intake initialized", Intake.getInstance()::getInitialized);
    diagnostics.addBoolean("Pivot initialized", Pivot.getInstance()::getInitialized);
    diagnostics.addBoolean("Shooter initialized", Shooter.getInstance()::getInitialized);
    diagnostics.addBoolean("Swerve initialized", Swerve.getInstance()::getInitialized);

    // Subsystem operating status
    diagnostics.addBoolean("Manager operable", m_Manager::checkSubsystem);
    diagnostics.addBoolean("Intake operable", Intake.getInstance()::checkSubsystem);
    diagnostics.addBoolean("Pivot operable", Pivot.getInstance()::checkSubsystem);
    diagnostics.addBoolean("Shooter operable", Shooter.getInstance()::checkSubsystem);
    diagnostics.addBoolean("Swerve operable", Swerve.getInstance()::checkSubsystem);

    // Current state of each subsystem
    diagnostics.addString("Robot state", () -> m_Manager.getState().toString());
    diagnostics.addString("Intake state", () -> Intake.getInstance().getState().toString());
    diagnostics.addString("Pivot state", () -> Pivot.getInstance().getState().toString());
    diagnostics.addString("Shooter state", () -> Shooter.getInstance().getState().toString());
    diagnostics.addString("Swerve state", () -> Swerve.getInstance().getState().toString());
  }

  private void configureAutos() {
    // To use in the Path Planner GUI
    NamedCommands.registerCommand("Run Shooter", new RunShooter(() -> 1.0, Shooter.getInstance()));
    NamedCommands.registerCommand("Pivot to Setpoint", new PIDPivotToSetpoint(() -> 0.1, () -> AutoConstants.kSpeakerMidPosition, Pivot.getInstance()));
    NamedCommands.registerCommand("Pivot flat", new PIDPivotToSetpoint(() -> 0.1, () -> 0.0, Pivot.getInstance()));
    NamedCommands.registerCommand("Run Intake", new RunIntake(() -> 0.8, Intake.getInstance()));
    NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> Intake.getInstance().setSpeed(0), Intake.getInstance()));
    NamedCommands.registerCommand("Run Intake - proximity sensor", new RunIntake(() -> 0.8, Intake.getInstance()).onlyWhile(proximityTrigger));
    NamedCommands.registerCommand("Stop drivetrain", new RunCommand(() -> Swerve.getInstance().drive(0,0,0,false,false,0), Swerve.getInstance()));
    // NamedCommands.registerCommand("gyro to 240", new RunCommand(() -> Swerve.getInstance().m_gyro.setYaw(240), Swerve.getInstance()));
    // NamedCommands.registerCommand("gyro to 180", new RunCommand(() -> Swerve.getInstance().m_gyro.setYaw(180), Swerve.getInstance()));
    // NamedCommands.registerCommand("gyro to 120", new RunCommand(() -> Swerve.getInstance().m_gyro.setYaw(120), Swerve.getInstance()));

    // Will immediantly crash code even though this is how the documenation recommends to do it
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
            () -> Swerve.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            Swerve.getInstance()).withTimeout(2),
          new Defense(() -> tx.get(), Swerve.getInstance())
        ));
    autoChooser.addOption("PID Defense", new SequentialCommandGroup(
          new RunCommand(
            () -> Swerve.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            Swerve.getInstance()).withTimeout(2),
          new PIDDefense(() -> tx.get(), Swerve.getInstance())
        ));

    // Hard coded autos
    autoChooser.addOption("Nothing", new InstantCommand());
    autoChooser.addOption("One note mid", getOnePieceAuto());
    autoChooser.addOption("One note side", getOnePieceAuto());
    autoChooser.addOption("manual blue interference auto", new SequentialCommandGroup(
        new RunCommand(
            () -> Swerve.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            Swerve.getInstance()).withTimeout(2),
        new Interference(true, Swerve.getInstance()).withTimeout(1.9)
    ));
    autoChooser.addOption("manual red interference auto", new SequentialCommandGroup(
        new RunCommand(
            () -> Swerve.getInstance().drive(0.76257,0,0,true, false, TeleopConstants.kSwerveSpeed),
            Swerve.getInstance()).withTimeout(2),
        new Interference(false, Swerve.getInstance()).withTimeout(1.9)
    ));

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
   * Runs the selected Instant command that sets the gyro angle.
   */
  public void configureGyro() {
    angleChooser.getSelected().schedule();
  }

  /**
   * Make sure to tune your pipelines at every event. This is to ensure you get accurate readings
   * under varying lighting conditions.
   *
   * Configures the intake limelight to pipeline zero if
   * on the blue alliance and pipeline one if on the red alliance.
   *
   * Configures the shooter limelight to pipeline zero if
   * on the blue alliance and pipeline one if on the red alliance.
   */
  public void configureLimelights() {
    // TODO make each pipeline only look for one specific Apriltag
    if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      LimelightHelpers.setPipelineIndex("limelight-shooter", 0);
    } else {
      LimelightHelpers.setPipelineIndex("limelight-shooter", 1);
    }

    if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
      LimelightHelpers.setPipelineIndex("limelight-intake", 0);
    } else {
      LimelightHelpers.setPipelineIndex("limelight-intake", 1);
    }
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
