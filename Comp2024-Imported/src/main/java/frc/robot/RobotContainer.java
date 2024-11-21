// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FDConsts.FDRollerMode;
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.Constants.VIConsts;
import frc.robot.autos.AutoLeave;
import frc.robot.autos.AutoPreloadCLine;
import frc.robot.autos.AutoPreloadLeave;
import frc.robot.autos.AutoPreloadScore;
import frc.robot.autos.AutoPreloadSteal;
import frc.robot.autos.AutoScore4;
import frc.robot.autos.AutoTest;
import frc.robot.commands.AcquireNote;
import frc.robot.commands.ExpelNote;
import frc.robot.commands.HandoffToFeeder;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PassNote;
import frc.robot.commands.PrepareToClimb;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.Vision;

/****************************************************************************
 * 
 * This class is where the bulk of the robot is declared. Since Command-based is a "declarative"
 * paradigm, very little robot logic should actually be handled in the Robot periodic methods (other
 * than the scheduler calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer
{
  private final boolean                               m_macOSXSim     = false;        // Enables Mac OS X controller compatibility in simulation
  private static final String                         kAutoTab        = "Autonomous"; // Shuffleboard tab name for autonomous mode
  //private static final String                         kCommandTab     = "Command";    // Shuffleboard tab name for commands

  // Gamepad controllers
  private static final CommandXboxController          m_driverPad     = new CommandXboxController(Constants.kDriverPadPort);
  private static final CommandXboxController          m_operatorPad   = new CommandXboxController(Constants.kOperatorPadPort);

  private static final double                         kMaxSpeed       = TunerConstants.kSpeedAt12VoltsMps; // Maximum top speed
  private static final double                         kMaxAngularRate = 3.0 * Math.PI;                     // 1.5 rotations per second max angular velocity
  private Command                                     m_autoCommand;

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric            drive           = new SwerveRequest.FieldCentric( ) //
      .withDeadband(kMaxSpeed * Constants.kStickDeadband)                  //
      .withRotationalDeadband(kMaxAngularRate * Constants.kStickDeadband)  //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);             // We want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake        brake           = new SwerveRequest.SwerveDriveBrake( );
  private final SwerveRequest.FieldCentricFacingAngle facing          = new SwerveRequest.FieldCentricFacingAngle( )  //
      .withDeadband(kMaxSpeed * Constants.kStickDeadband)                  //
      .withRotationalDeadband(kMaxAngularRate * Constants.kStickDeadband)  //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);             // We want field-centric driving in open loop
  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt           point           = new SwerveRequest.PointWheelsAt( );
  private final SwerveRequest.RobotCentric            aim             = new SwerveRequest.RobotCentric( );
  private final SwerveRequest.Idle                    idle            = new SwerveRequest.Idle( );

  private final Telemetry                             logger          = new Telemetry(kMaxSpeed);

  // The robot's shared subsystems
  private final HID                                   m_hid           = new HID(m_driverPad.getHID( ), m_operatorPad.getHID( ));
  private final LED                                   m_led           = new LED( );
  private final Power                                 m_power         = new Power( );
  private final Vision                                m_vision        = new Vision( );

  // These subsystems may use LED or vision and must be created afterward
  private final CommandSwerveDrivetrain               m_drivetrain    = TunerConstants.DriveTrain;
  private final Intake                                m_intake        = new Intake( );
  private final Shooter                               m_shooter       = new Shooter( );
  private final Feeder                                m_feeder        = new Feeder( );
  private final Climber                               m_climber       = new Climber( );

  Command                                             m_autoCmd;

  /**
   * Chooser options for autonomous commands - all starting from poses 1-3
   */
  private enum AutoChooser
  {
    AUTOSTOP,         // AutoStop - sit still, do nothing
    AUTOLEAVE,        // Leave starting zone avoiding spikes
    AUTOPRELOADLEAVE, // Score preload at waypoints P0, P2, and P4 and leave starting zone
    AUTOPRELOADSCORE, // Score preload at waypoints P1-P3 and score another from nearest spike
    AUTOPRELOADSTEAL, // Score preload at waypoints P1-P3 and steal centerline notes
    AUTOSCORE4,       // Score preload at waypoints P1-P3 and spike notes at S1-S3
    AUTOPRELOADCLINE, // Score preload at waypoints P0, P2, and P4 and score C1-C5 notes
    AUTOTEST          // Run a selected test auto
  }

  /**
   * Chooser options for autonomous starting pose to select pose 1-3
   */
  private enum StartPose
  {
    POSE1, // Starting pose 1 - blue left, red right (driver perspective)
    POSE2, // Starting pose 2 - blue/red middle (driver perspective)
    POSE3  // Starting pose 3 - blue right, red left (driver perspective)
  }

  /** Dashboard chooser for auto option selection */
  private SendableChooser<AutoChooser>  m_autoChooser  = new SendableChooser<>( );
  /** Dashboard chooser for starting pose selection */
  private SendableChooser<StartPose>    m_startChooser = new SendableChooser<>( );

  /**
   * Hash map of autonomous option relations to auto filenames
   * 
   * @param key
   *          the auto option and pose selected
   * @param value
   *          the auto filename associated with the key
   */
  private final HashMap<String, String> autoMap        = new HashMap<>(Map.ofEntries( //
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.POSE1.toString( ), "Pos1_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.POSE2.toString( ), "Pos2_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.POSE3.toString( ), "Pos3_Stop"),

      Map.entry(AutoChooser.AUTOLEAVE.toString( ) + StartPose.POSE1.toString( ), "Pos1_L1"),
      Map.entry(AutoChooser.AUTOLEAVE.toString( ) + StartPose.POSE2.toString( ), "Pos2_L2"),
      Map.entry(AutoChooser.AUTOLEAVE.toString( ) + StartPose.POSE3.toString( ), "Pos3_L3"),

      Map.entry(AutoChooser.AUTOPRELOADLEAVE.toString( ) + StartPose.POSE1.toString( ), "Pos1_P0_L0"),
      Map.entry(AutoChooser.AUTOPRELOADLEAVE.toString( ) + StartPose.POSE2.toString( ), "Pos2_P2_L2"),
      Map.entry(AutoChooser.AUTOPRELOADLEAVE.toString( ) + StartPose.POSE3.toString( ), "Pos3_P4_L4"),

      Map.entry(AutoChooser.AUTOPRELOADSCORE.toString( ) + StartPose.POSE1.toString( ), "Pos1_P1_S1_P1"),
      Map.entry(AutoChooser.AUTOPRELOADSCORE.toString( ) + StartPose.POSE2.toString( ), "Pos2_P2_S2_P2"),
      Map.entry(AutoChooser.AUTOPRELOADSCORE.toString( ) + StartPose.POSE3.toString( ), "Pos3_P3_S3_P3"),

      Map.entry(AutoChooser.AUTOPRELOADSTEAL.toString( ) + StartPose.POSE1.toString( ), "Pos1_P0_C1_F1_C2_F1"),
      Map.entry(AutoChooser.AUTOPRELOADSTEAL.toString( ) + StartPose.POSE2.toString( ), "Pos2_P2_C5_F5_C4_F5"),
      Map.entry(AutoChooser.AUTOPRELOADSTEAL.toString( ) + StartPose.POSE3.toString( ), "Pos3_P4_C5_F5_C4_F5"),

      Map.entry(AutoChooser.AUTOSCORE4.toString( ) + StartPose.POSE1.toString( ), "Pos1_P1_S1_P1_S2_P2_S3_P3"),
      Map.entry(AutoChooser.AUTOSCORE4.toString( ) + StartPose.POSE2.toString( ), "Pos2_P1_S1_P1_S2_P2_S3_P3"),
      Map.entry(AutoChooser.AUTOSCORE4.toString( ) + StartPose.POSE3.toString( ), "Pos3_P3_S3_P3_S2_P2_S1_P1"),

      Map.entry(AutoChooser.AUTOPRELOADCLINE.toString( ) + StartPose.POSE1.toString( ), "Pos1_P0_C1_P0_C2_P0"),
      Map.entry(AutoChooser.AUTOPRELOADCLINE.toString( ) + StartPose.POSE2.toString( ), "Pos2_P2_C5_P4_C4_P4"),
      Map.entry(AutoChooser.AUTOPRELOADCLINE.toString( ) + StartPose.POSE3.toString( ), "Pos3_P4_C5_P4_C4_P4"),

      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.POSE1.toString( ), "Pos1_test1"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.POSE2.toString( ), "Pos2_test2"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.POSE3.toString( ), "Pos3_test3") //
  ));

  // Shuffleboard objects
  private ShuffleboardTab               autoTab        = Shuffleboard.getTab(kAutoTab);
  private SimpleWidget                  autoDelay      = autoTab.add("AutoDelay", 0.0).withPosition(6, 2).withSize(2, 1);

  /****************************************************************************
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer( )
  {
    Robot.timeMarker("robotContainer: before DAQ thread");

    m_drivetrain.getDaqThread( ).setThreadPriority(99);                   // Start swerve telemetry thread
    facing.HeadingController = new PhoenixPIDController(10.0, 0.0, 0.0);  // Swerve steer PID for facing request

    addDashboardWidgets( );           // Add dashboard widgets for commands

    configureButtonBindings( );       // Configure game controller buttons

    initDefaultCommands( );           // Initialize subsystem default commands

    Robot.timeMarker("robotContainer: after default commands");
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addDashboardWidgets( )
  {
    // Set up Shuffleboard layout from code
    autoTab.add("AutoMode", m_autoChooser).withPosition(6, 0).withSize(2, 1);
    autoTab.add("StartPosition", m_startChooser).withPosition(6, 1).withSize(2, 1);

    // Configure autonomous sendable chooser
    m_autoChooser.setDefaultOption("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.addOption("1 - AutoLeave", AutoChooser.AUTOLEAVE);
    m_autoChooser.addOption("2 - AutoPreloadLeave", AutoChooser.AUTOPRELOADLEAVE);
    m_autoChooser.addOption("3 - AutoPreloadScore", AutoChooser.AUTOPRELOADSCORE);
    m_autoChooser.addOption("4 - AutoPreloadSteal", AutoChooser.AUTOPRELOADSTEAL);
    m_autoChooser.addOption("5 - AutoScore4", AutoChooser.AUTOSCORE4);
    m_autoChooser.addOption("6 - AutoPreloadCLine", AutoChooser.AUTOPRELOADCLINE);
    m_autoChooser.addOption("7 - AutoTestPath", AutoChooser.AUTOTEST);

    // Configure starting pose sendable chooser
    m_startChooser.setDefaultOption("POSE1", StartPose.POSE1);
    m_startChooser.addOption("POSE2", StartPose.POSE2);
    m_startChooser.addOption("POSE3", StartPose.POSE3);

    autoTab.add("AutoChooserRun", new InstantCommand(( ) -> getAutonomousCommand( ))).withPosition(6, 2);

    // Command tab
    // ShuffleboardTab cmdTab = Shuffleboard.getTab(kCommandTab);
    // cmdTab.add("AcquireNote", new AcquireNote(m_intake, m_led, m_hid));
    // cmdTab.add("ExpelNote", new ExpelNote(m_intake, m_led));
    // cmdTab.add("HandoffToFeeder", new HandoffToFeeder(m_intake, m_feeder, m_led));

    // cmdTab.add("RetractIntake", new RetractIntake(m_intake, m_led, m_hid));
    // cmdTab.add("ScoreAmp", new ScoreAmp(m_feeder));
    // cmdTab.add("ScoreSpeaker", new ScoreSpeaker(m_shooter, m_intake, m_led));

    // cmdTab.add("HIDRumbleDriver", m_hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Constants.kRumbleIntensity));
    // cmdTab.add("HIDRumbleOperator", m_hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Constants.kRumbleIntensity));
    // cmdTab.add("PrepareToClimb", new PrepareToClimb(m_climber, m_feeder));

    // ShuffleboardLayout subList = cmdTab.getLayout("Subsystems", BuiltInLayouts.kList).withProperties(Map.of("Label position", "HIDDEN"));
    // subList.add(m_intake);
    // subList.add(m_shooter);
    // subList.add(m_feeder);
    // subList.add(m_climber);

    // cmdTab.add(CommandScheduler.getInstance( ));
  }

  /****************************************************************************
   * 
   * Define button-command mappings. Buttons are created by instantiating a GenericHID or one of its
   * subclasses (Joystick or XboxController), and then passing it to a JoystickButton.
   */
  private void configureButtonBindings( )
  {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    //
    m_driverPad.a( ).whileTrue(m_drivetrain.applyRequest(( ) -> aim                 //
        .withVelocityX(-m_vision.limelight_range_proportional(kMaxSpeed))            //
        .withVelocityY(0)                                                 //
        .withRotationalRate(m_vision.limelight_aim_proportional(kMaxAngularRate))));
    m_driverPad.b( ).onTrue(new LogCommand("driverPad", "B")); // drive to stage right
    m_driverPad.x( ).onTrue(new LogCommand("driverPad", "X")); // drive to stage left
    m_driverPad.y( ).onTrue(new LogCommand("driverPad", "Y")); // drive to stage center

    //
    // Driver - Bumpers, start, back
    //
    m_driverPad.leftBumper( ).whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kAmpPose));  // drive to amp
    m_driverPad.rightBumper( ).onTrue(new AcquireNote(m_intake, m_led, m_hid));
    m_driverPad.rightBumper( ).onFalse(new RetractIntake(m_intake, m_led, m_hid));
    m_driverPad.back( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));                       // aka View
    m_driverPad.start( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldRelative( )));  // aka Menu

    //
    // Driver - POV buttons
    //
    m_driverPad.pov(0).whileTrue(m_drivetrain.applyRequest(( ) -> facing //
        .withVelocityX(-m_driverPad.getLeftY( ) * kMaxSpeed)  //
        .withVelocityY(-m_driverPad.getLeftX( ) * kMaxSpeed)  //
        .withTargetDirection(Rotation2d.fromDegrees(0.0))));
    m_driverPad.pov(90).whileTrue(m_drivetrain.applyRequest(( ) -> facing //
        .withVelocityX(-m_driverPad.getLeftY( ) * kMaxSpeed)  //
        .withVelocityY(-m_driverPad.getLeftX( ) * kMaxSpeed)  //
        .withTargetDirection(Rotation2d.fromDegrees(270.0))));
    m_driverPad.pov(180).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(-m_driverPad.getLeftY( ) * kMaxSpeed)  //
        .withVelocityY(-m_driverPad.getLeftX( ) * kMaxSpeed)  //
        .withTargetDirection(Rotation2d.fromDegrees(180.0))));
    m_driverPad.pov(270).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(-m_driverPad.getLeftY( ) * kMaxSpeed)  //
        .withVelocityY(-m_driverPad.getLeftX( ) * kMaxSpeed)  //
        .withTargetDirection(Rotation2d.fromDegrees(90.0))));

    //
    // Driver Left/Right Trigger
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_driverPad.leftTrigger(Constants.kTriggerThreshold)
        .whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kSpeakerPose));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ScoreSpeaker(m_shooter, m_intake, m_led));

    m_driverPad.leftStick( ).onTrue(new LogCommand("driverPad", "left stick"));
    m_driverPad.rightStick( ).onTrue(new LogCommand("driverPad", "right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    //
    m_operatorPad.a( ).onTrue(m_shooter.getShooterScoreCommand( ));
    m_operatorPad.b( ).onTrue(m_shooter.getShooterStopCommand( ));
    m_operatorPad.x( ).onTrue(new PassNote(m_shooter, m_intake, m_led));
    m_operatorPad.x( ).onFalse(m_shooter.getShooterScoreCommand( ));
    m_operatorPad.y( ).onTrue(new ExpelNote(m_intake, m_led));

    //
    // Operator - Bumpers, start, back
    //
    m_operatorPad.leftBumper( ).onTrue(new HandoffToFeeder(m_intake, m_feeder, m_led));
    m_operatorPad.rightBumper( ).onTrue(new AcquireNote(m_intake, m_led, m_hid));
    m_operatorPad.rightBumper( ).onFalse(new RetractIntake(m_intake, m_led, m_hid));
    m_operatorPad.back( ).toggleOnTrue(m_climber.getJoystickCommand(( ) -> getClimberAxis( )));  // aka View
    m_operatorPad.start( ).onTrue(new InstantCommand(m_vision::rotateCameraStreamMode).ignoringDisable(true)); // aka Menu

    //
    // Operator - POV buttons
    //
    m_operatorPad.pov(0).onTrue(new PrepareToClimb(m_climber, m_feeder));
    m_operatorPad.pov(90).onTrue(new LogCommand("operPad", "POV 90"));
    m_operatorPad.pov(180).onTrue(m_climber.getMoveToPositionCommand(m_climber::getClimberClimbed));
    m_operatorPad.pov(270).onTrue(m_climber.getMoveToPositionCommand(m_climber::getClimberChainLevel));

    //
    // Operator Left/Right Trigger
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new ScoreAmp(m_feeder));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ScoreSpeaker(m_shooter, m_intake, m_led));

    m_operatorPad.leftStick( ).toggleOnTrue(m_feeder.getJoystickCommand(( ) -> getFeederAxis( )));
    m_operatorPad.rightStick( ).toggleOnTrue(m_intake.getJoystickCommand(( ) -> getIntakeAxis( )));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    if (!m_macOSXSim)
    {
      m_drivetrain.setDefaultCommand(                                                  // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                       //
              .withVelocityX(-m_driverPad.getLeftY( ) * kMaxSpeed)                     // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * kMaxSpeed)                     // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getRightX( ) * kMaxAngularRate)         // Drive counterclockwise with negative X (left)
          )                                                                            //
              .ignoringDisable(true)                               //
              .withName("CommandSwerveDrivetrain"));
    }
    else // When using simulation on MacOS X, XBox controllers need to be re-mapped due to an Apple bug
    {
      m_drivetrain.setDefaultCommand(                                                   // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                        //
              .withVelocityX(-m_driverPad.getLeftY( ) * kMaxSpeed)                      // Drive forward with negative Y (forward)
              .withVelocityY(-m_driverPad.getLeftX( ) * kMaxSpeed)                      // Drive left with negative X (left)
              .withRotationalRate(-m_driverPad.getLeftTriggerAxis( ) * kMaxAngularRate) // Drive counterclockwise with negative X (left)
          )                                                                             //
              .ignoringDisable(true)                                //
              .withName("CommandSwerveDrivetrain"));
    }

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // Default command - Motion Magic hold
    m_intake.setDefaultCommand(m_intake.getHoldPositionCommand(INRollerMode.HOLD, m_intake::getCurrentPosition));
    m_feeder.setDefaultCommand(m_feeder.getHoldPositionCommand(FDRollerMode.HOLD, m_feeder::getCurrentPosition));
    m_climber.setDefaultCommand(m_climber.getHoldPositionCommand(m_climber::getClimberPosition));

    //Default command - manual mode
    // m_intake.setDefaultCommand(m_intake.getJoystickCommand(( ) -> getIntakeAxis( )));
    // m_feeder.setDefaultCommand(m_feeder.getJoystickCommand(( ) -> getFeederAxis( )));
    // m_climber.setDefaultCommand(m_climber.getJoystickCommand(( ) -> getClimberAxis( )));
  }

  /****************************************************************************
   * 
   * Use this to pass the autonomous command to the main Robot class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand( )
  {
    AutoChooser autoOption = m_autoChooser.getSelected( );
    StartPose startOption = m_startChooser.getSelected( );
    String autoKey = autoOption.toString( ) + startOption.toString( );

    if (m_autoCommand != null)
    {
      if (m_autoCommand.isScheduled( ))
        m_autoCommand.cancel( );
      m_autoCommand = null;
    }

    // Get auto value using created key
    String autoName = autoMap.get(autoKey);
    DataLogManager.log(String.format("===================================================================="));
    DataLogManager.log(String.format("getAuto: autoKey: %s  autoName: %s", autoKey, autoName));
    DataLogManager.log(String.format("===================================================================="));

    // If auto not defined in hashmap, no path assigned so sit idle
    if (autoName == null)
    {
      DataLogManager.log(String.format("getAuto: ERROR - no auto defined for this autoKey (%s)", autoKey));
      return m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
    }

    // Get list of paths within the auto
    List<PathPlannerPath> ppPathList = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    if (ppPathList.isEmpty( ))
    {
      DataLogManager.log(String.format("getAuto: ERROR - auto path list is empty"));
      return m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
    }

    DataLogManager.log(String.format("getAuto: %s contains %s paths in list", autoName, ppPathList.size( )));

    // If on red alliance, flip each path
    PathPlannerPath initialPath = ppPathList.get(0);
    if (DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red)
      initialPath = initialPath.flipPath( );

    // { // Debug only: print states of first path
    //   List<PathPlannerTrajectory.State> states = initialPath.getTrajectory(new ChassisSpeeds( ), new Rotation2d( )).getStates( );
    //   for (int i = 0; i < states.size( ); i++)
    //     DataLogManager.log(String.format("autoCommand: Auto path state: (%d) %s", i, states.get(i).getTargetHolonomicPose( )));
    // }

    // Set field centric robot position to start of auto sequence
    Pose2d startPose = initialPath.getPreviewStartingHolonomicPose( );
    if (startPose != null)
      m_drivetrain.seedFieldRelative(startPose);

    // Create the correct base command and pass the path list
    switch (autoOption)
    {
      default :
      case AUTOSTOP :
        m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
        break;
      case AUTOLEAVE :
        m_autoCommand = new AutoLeave(ppPathList, m_drivetrain, m_led);
        break;
      case AUTOPRELOADLEAVE :
        m_autoCommand = new AutoPreloadLeave(ppPathList, m_drivetrain, m_intake, m_shooter, m_led);
        break;
      case AUTOPRELOADSCORE :
        m_autoCommand = new AutoPreloadScore(ppPathList, m_drivetrain, m_intake, m_shooter, m_led, m_hid);
        break;
      case AUTOPRELOADSTEAL :
        m_autoCommand = new AutoPreloadSteal(ppPathList, m_drivetrain, m_intake, m_shooter, m_led, m_hid);
        break;
      case AUTOSCORE4 :
        m_autoCommand = new AutoScore4(ppPathList, m_drivetrain, m_intake, m_shooter, m_led, m_hid);
        break;
      case AUTOPRELOADCLINE :
        m_autoCommand = new AutoPreloadCLine(ppPathList, m_drivetrain, m_intake, m_shooter, m_led, m_hid);
        break;
      case AUTOTEST :
        m_autoCommand = new AutoTest(ppPathList, m_drivetrain, m_led);
        break;
    }

    DataLogManager.log(String.format("getAuto: autoMode %s startOption %s (%s)", autoKey, startPose, m_autoCommand.getName( )));

    double delay = autoDelay.getEntry( ).getDouble(0.0);
    if (delay > 0.0)
      m_autoCommand = new SequentialCommandGroup( //
          new LogCommand("Autodelay", String.format("Delaying %.1f seconds ...", delay)), //
          new WaitCommand(delay), //
          m_autoCommand);

    return m_autoCommand;
  }

  /****************************************************************************
   * 
   * Gamepad interfaces
   */
  public double getIntakeAxis( )
  {
    return m_operatorPad.getRightX( );
  }

  public double getFeederAxis( )
  {
    return m_operatorPad.getLeftX( );
  }

  public double getClimberAxis( )
  {
    return -m_operatorPad.getRightY( );
  }

  /****************************************************************************
   * 
   * Called by disabledInit - place subsystem initializations here
   */
  public void initialize( )
  {
    m_led.initialize( );
    m_power.initialize( );
    m_vision.initialize( );

    m_intake.initialize( );
    m_shooter.initialize( );
    m_feeder.initialize( );
    m_climber.initialize( );
  }

  /****************************************************************************
   * 
   * Called when user button is pressed - place subsystem fault dumps here
   */
  public void printFaults( )
  {
    m_led.printFaults( );
    m_power.printFaults( );

    m_intake.printFaults( );
    m_shooter.printFaults( );
    m_feeder.printFaults( );
    m_climber.printFaults( );
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void autoInit( )
  {
    CommandScheduler.getInstance( ).schedule(m_climber.getCalibrateCommand( ));
    CommandScheduler.getInstance( ).schedule(m_shooter.getShooterScoreCommand( ));
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void teleopInit( )
  {
    CommandScheduler.getInstance( ).schedule(m_climber.getCalibrateCommand( ));
    CommandScheduler.getInstance( ).schedule(m_shooter.getShooterScoreCommand( ));
  }

}
