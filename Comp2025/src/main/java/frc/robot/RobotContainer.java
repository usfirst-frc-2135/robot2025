// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VIConsts;
import frc.robot.commands.LogCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/****************************************************************************
 * 
 * This class is where the bulk of the robot is declared. Since Command-based is
 * a "declarative"
 * paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other
 * than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer {
  private final boolean m_macOSXSim = false; // Enables Mac OS X controller compatibility in simulation

  // Gamepad controllers
  private static final CommandXboxController m_driverPad = new CommandXboxController(Constants.kDriverPadPort);
  private static final CommandXboxController m_operatorPad = new CommandXboxController(Constants.kOperatorPadPort);

  private static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts; // Maximum top speed
  private static final AngularVelocity kMaxAngularRate = RadiansPerSecond.of(3.0 * Math.PI); // Max 1.5 rot per second

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() //
      .withDeadband(kMaxSpeed.times(Constants.kStickDeadband)) //
      .withRotationalDeadband(kMaxAngularRate.times(Constants.kStickDeadband)) //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // We want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.FieldCentricFacingAngle facing = new SwerveRequest.FieldCentricFacingAngle() //
      .withDeadband(kMaxSpeed.times(Constants.kStickDeadband)) //
      .withRotationalDeadband(kMaxAngularRate.times(Constants.kStickDeadband)) //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // We want field-centric driving in open loop
  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric aim = new SwerveRequest.RobotCentric();
  private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
  @SuppressWarnings("unused")
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // The robot's shared subsystems

  // These subsystems may use LED or vision and must be created afterward
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final Climber m_climber = new Climber();

  private Command m_autoCommand; // Selected autonomous command

  /**
   * Chooser options for autonomous commands - all starting from poses 1-3
   */
  private enum AutoChooser {
    AUTOSTOP, // AutoStop - sit still, do nothing
    AUTOLEAVE, // Leave starting zone avoiding spikes
    AUTOPRELOADLEAVE, // Score preload at waypoints P0, P2, and P4 and leave starting zone
    AUTOPRELOADSCORE, // Score preload at waypoints P1-P3 and score another from nearest spike
    AUTOSCORE4, // Score preload at waypoints P1-P3 and all spike notes at S1-S3
    AUTOPRELOADSTEAL, // Score preload at waypoints P0, P2, and P4 and steal centerline notes
    AUTOPRELOADCLINE, // Score preload at waypoints P0, P2, and P4 and score C1-C5 notes
    AUTOTEST // Run a selected test auto
  }

  /**
   * Chooser options for autonomous starting pose to select pose 1-3
   */
  private enum StartPose {
    POSE1, // Starting pose 1 - blue left, red right (driver perspective)
    POSE2, // Starting pose 2 - blue/red middle (driver perspective)
    POSE3 // Starting pose 3 - blue right, red left (driver perspective)
  }

  /** Dashboard chooser for auto option selection */
  private SendableChooser<AutoChooser> m_autoChooser = new SendableChooser<>();
  /** Dashboard chooser for starting pose selection */
  private SendableChooser<StartPose> m_startChooser = new SendableChooser<>();

  /**
   * Hash map of autonomous option relations to auto filenames
   * 
   * @param key
   *              the auto option and pose selected
   * @param value
   *              the auto filename associated with the key
   */
  private final HashMap<String, String> autoMap = new HashMap<>(Map.ofEntries( //

  ));

  /****************************************************************************
   * 
   * The main container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    Robot.timeMarker("robotContainer: before DAQ thread");

    facing.HeadingController = new PhoenixPIDController(10.0, 0.0, 0.0); // Swerve steer PID for facing swerve request

    addDashboardWidgets(); // Add dashboard widgets for commands

    configureButtonBindings(); // Configure game controller buttons

    initDefaultCommands(); // Initialize subsystem default commands

    Robot.timeMarker("robotContainer: after default commands");
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addDashboardWidgets() {
    // Network tables publisher objects
    SmartDashboard.putData("AutoMode", m_autoChooser);
    SmartDashboard.putData("StartPosition", m_startChooser);
    SmartDashboard.putNumber("AutoDelay", 0.0);

    // Configure autonomous sendable chooser
    m_autoChooser.setDefaultOption("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.addOption("1 - AutoLeave", AutoChooser.AUTOLEAVE);
    m_autoChooser.addOption("2 - AutoPreloadLeave", AutoChooser.AUTOPRELOADLEAVE);
    m_autoChooser.addOption("3 - AutoPreloadScore", AutoChooser.AUTOPRELOADSCORE);
    m_autoChooser.addOption("4 - AutoScore4", AutoChooser.AUTOSCORE4);
    m_autoChooser.addOption("5 - AutoPreloadSteal", AutoChooser.AUTOPRELOADSTEAL);
    m_autoChooser.addOption("6 - AutoPreloadCLine", AutoChooser.AUTOPRELOADCLINE);
    m_autoChooser.addOption("7 - AutoTestPath", AutoChooser.AUTOTEST);

    // Configure starting pose sendable chooser
    m_startChooser.setDefaultOption("POSE1", StartPose.POSE1);
    m_startChooser.addOption("POSE2", StartPose.POSE2);
    m_startChooser.addOption("POSE3", StartPose.POSE3);

    // Command tab

    // Network tables publisher objects

    SmartDashboard.putData("climber", m_climber);

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /****************************************************************************
   * 
   * Define button-command mappings. Triggers are created and bound to the desired
   * commands.
   */
  private void configureButtonBindings() {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    //
    // --- Normal button definitions ---
    //

    //
    // --- SysId button definitions ---
    //
    // Run SysId routines when holding A, B and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_driverPad.a( ).and(m_driverPad.y(
    // )).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverPad.a( ).and(m_driverPad.x(
    // )).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverPad.b( ).and(m_driverPad.y(
    // )).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverPad.b( ).and(m_driverPad.x(
    // )).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    //
    // Driver - Bumpers, start, back
    //
    m_driverPad.leftBumper().whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kAmpPose)); // drive to amp
    // m_driverPad.rightBumper().onTrue(new AcquireNote(m_intake, m_led, m_hid));
    // m_driverPad.rightBumper().onFalse(new RetractIntake(m_intake, m_led, m_hid));
    m_driverPad.back().whileTrue(m_drivetrain.applyRequest(() -> brake)); // aka View button
    m_driverPad.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric())); // aka Menu button

    //
    // Driver - POV buttons
    //
    m_driverPad.pov(0).whileTrue(m_drivetrain.applyRequest(() -> facing //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY())) //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX())) //
        .withTargetDirection(Rotation2d.fromDegrees(0.0))));
    m_driverPad.pov(90).whileTrue(m_drivetrain.applyRequest(() -> facing //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY())) //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX())) //
        .withTargetDirection(Rotation2d.fromDegrees(270.0))));
    m_driverPad.pov(180).whileTrue(m_drivetrain.applyRequest(() -> facing //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY())) //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX())) //
        .withTargetDirection(Rotation2d.fromDegrees(180.0))));
    m_driverPad.pov(270).whileTrue(m_drivetrain.applyRequest(() -> facing //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY())) //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX())) //
        .withTargetDirection(Rotation2d.fromDegrees(90.0))));

    //
    // Driver Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX
    // = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger =
    // 5, rightTrigger = 4}
    //
    m_driverPad.leftTrigger(Constants.kTriggerThreshold)
        .whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kSpeakerPose));

    m_driverPad.leftStick().onTrue(new LogCommand("driverPad", "left stick"));
    m_driverPad.rightStick().onTrue(new LogCommand("driverPad", "right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y

    //
    // Operator - Bumpers, start, back
    //

    m_operatorPad.back().toggleOnTrue(m_climber.getJoystickCommand(() -> getClimberAxis())); // aka View button
    // aka Menu button

    //
    // Operator - POV buttons
    //
    // m_operatorPad.pov(0).onTrue(new PrepareToClimb(m_climber, m_feeder));
    m_operatorPad.pov(90).onTrue(new LogCommand("operPad", "POV 90"));
    m_operatorPad.pov(180).onTrue(m_climber.getMoveToPositionCommand(m_climber::getClimberClimbed));
    m_operatorPad.pov(270).onTrue(m_climber.getMoveToPositionCommand(m_climber::getClimberChainLevel));

    //
    // Operator Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX
    // = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger =
    // 5, rightTrigger = 4}

  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands() {
    if (!m_macOSXSim) {
      m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(() -> drive //
              .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY())) // Drive forward with negative Y (forward)
              .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX())) // Drive left with negative X (left)
              .withRotationalRate(kMaxAngularRate.times(-m_driverPad.getRightX())) // Drive counterclockwise with
                                                                                   // negative X (left)
          ) //
              .ignoringDisable(true) //
              .withName("CommandSwerveDrivetrain"));
    } else // When using simulation on MacOS X, XBox controllers need to be re-mapped due
           // to an Apple bug
    {
      m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(() -> drive //
              .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY())) // Drive forward with negative Y (forward)
              .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX())) // Drive left with negative X (left)
              .withRotationalRate(kMaxAngularRate.times(-m_driverPad.getLeftTriggerAxis())) // Drive counterclockwise
                                                                                            // with negative X (left)
          ) //
              .ignoringDisable(true) //
              .withName("CommandSwerveDrivetrain"));
    }

    // m_drivetrain.registerTelemetry(logger::telemeterize);

    // Default command - Motion Magic hold

    m_climber.setDefaultCommand(m_climber.getHoldPositionCommand(m_climber::getClimberPosition));

    // Default command - manual mode

    m_climber.setDefaultCommand(m_climber.getJoystickCommand(() -> getClimberAxis()));
  }

  /****************************************************************************
   * 
   * Use this to pass the autonomous command to the main Robot class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // AutoChooser autoOption = m_autoChooser.getSelected();

  // String autoKey = autoOption.toString() + startOption.toString();

  // }

  // Get auto value using created key

  // If auto not defined in hashmap, no path assigned so sit idle

  // Get list of paths within the auto

  // If on red alliance, flip each path

  // Debug only: print states of first path
  // List<PathPlannerTrajectory.State> states = initialPath.getTrajectory(new
  // ChassisSpeeds( ), new Rotation2d( )).getStates( );
  // for (int i = 0; i < states.size( ); i++)
  // DataLogManager.log(String.format("autoCommand: Auto path state: (%d) %s", i,
  // states.get(i).getTargetHolonomicPose( )));
  // }

  // Set field centric robot position to start of auto sequence

  // Create the correct base command and pass the path list

  /****************************************************************************
   * 
   * Gamepad joystick axis interfaces
   */

  public double getClimberAxis() {
    return -m_operatorPad.getRightY();
  }

  /****************************************************************************
   * 
   * Called by disabledInit - place subsystem initializations here
   */
  public void initialize() {
    m_climber.initialize();
  }

  /****************************************************************************
   * 
   * Called when user button is pressed - place subsystem fault dumps here
   */
  public void printFaults() {
    m_climber.printFaults();
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void autoInit() {
    // CommandScheduler.getInstance( ).schedule(m_climber.getCalibrateCommand( ));

  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void teleopInit() {
    // CommandScheduler.getInstance( ).schedule(m_climber.getCalibrateCommand( ));

  }
}
