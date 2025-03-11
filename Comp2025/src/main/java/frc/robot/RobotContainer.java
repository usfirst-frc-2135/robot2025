// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.VIConsts;
import frc.robot.autos.AutoLeave;
import frc.robot.autos.AutoPreload;
import frc.robot.autos.AutoPreloadAlgae;
import frc.robot.autos.AutoPreloadAlgae2;
import frc.robot.autos.AutoPreloadCoral;
import frc.robot.autos.AutoPreloadCoral2;
import frc.robot.autos.AutoPreloadCoral3;
import frc.robot.autos.AutoTest;
import frc.robot.commands.AcquireAlgae;
import frc.robot.commands.AcquireCoral;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Power;
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
  private final boolean                               m_macOSXSim     = false;  // Enables Mac OS X controller compatibility in simulation

  // Gamepad controllers
  private static final CommandXboxController          m_driverPad     = new CommandXboxController(Constants.kDriverPadPort);
  private static final CommandXboxController          m_operatorPad   = new CommandXboxController(Constants.kOperatorPadPort);

  private static final LinearVelocity                 kMaxSpeed       = TunerConstants.kSpeedAt12Volts;     // Maximum top speed
  private static final AngularVelocity                kMaxAngularRate = RadiansPerSecond.of(3.0 * Math.PI); // Max 1.5 rot per second
  private static final double                         kSlowSwerve     = 0.35;                               // Throttle max swerve speeds for finer control
  private static final double                         kHeadingKp      = 10.0;
  private static final double                         kHeadingKi      = 0.0;
  private static final double                         kHeadingKd      = 0.0;

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric            drive           = new SwerveRequest.FieldCentric( ) //
      .withDeadband(kMaxSpeed.times(Constants.kStickDeadband))                  //
      .withRotationalDeadband(kMaxAngularRate.times(Constants.kStickDeadband))  //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                  // We want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake        brake           = new SwerveRequest.SwerveDriveBrake( );
  private final SwerveRequest.FieldCentricFacingAngle facing          = new SwerveRequest.FieldCentricFacingAngle( )  //
      .withDeadband(kMaxSpeed.times(Constants.kStickDeadband))                  //
      .withRotationalDeadband(kMaxAngularRate.times(Constants.kStickDeadband))  //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);                  // We want field-centric driving in open loop
  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt           point           = new SwerveRequest.PointWheelsAt( );
  // private final SwerveRequest.RobotCentric            aim             = new SwerveRequest.RobotCentric( );
  private final SwerveRequest.Idle                    idle            = new SwerveRequest.Idle( );
  @SuppressWarnings("unused")
  private final SwerveRequest.RobotCentric            forwardStraight =
      new SwerveRequest.RobotCentric( ).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry                             logger          = new Telemetry(kMaxSpeed.in(MetersPerSecond));

  // The robot's shared subsystems
  private final HID                                   m_hid           = new HID(m_driverPad.getHID( ), m_operatorPad.getHID( ));
  private final LED                                   m_led           = new LED( );
  private final Power                                 m_power         = new Power( );
  private final Vision                                m_vision        = new Vision( );

  // These subsystems may use LED or vision and must be created afterward
  private final CommandSwerveDrivetrain               m_drivetrain    = TunerConstants.createDrivetrain( );
  private final Elevator                              m_elevator      = new Elevator( );
  private final Manipulator                           m_manipulator   = new Manipulator( );

  // Selected autonomous command
  private Command                                     m_autoCommand;    // Selected autonomous command
  private IntegerPublisher                            m_reefLevelPub;   // Level of the reef to score or acquire from (1-4)
  private IntegerPublisher                            m_reefOffsetPub;  // Branch of the reef to score or acquire from (left, middle, right)

  /**
   * Chooser options for autonomous commands - all starting from poses 1-3
   */
  private enum AutoChooser
  {
    AUTOSTOP,           // AutoStop - sit still, do nothing
    AUTOLEAVE,          // Leave starting line
    AUTOPRELOAD,        // Preload coral
    AUTOPRELOADCORAL,   // Preload coral and score one more
    AUTOPRELOADCORAL2,  // Preload coral and score two more
    AUTOPRELOADCORAL3,  // Preload coral and score three more
    AUTOPRELOADALGAE,   // Preload coral and score one algae
    AUTOPRELOADALGAE2,  // Preload coral and score two algaes
    AUTOTEST            // Run a selected test auto
  }

  /**
   * Chooser options for autonomous starting pose to select pose 1-3 (driver perspective)
   */
  private enum StartPose
  {
    START1, // Starting pose 1 - leftmost aligned with a blue cage
    START2, // Starting pose 2 - middle aligned with reef
    START3  // Starting pose 3 - rightmost aligned with a red cage
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
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START1.toString( ), "Start1_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START2.toString( ), "Start2_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START3.toString( ), "Start3_Stop"),

      Map.entry(AutoChooser.AUTOLEAVE.toString( ) + StartPose.START1.toString( ), "Start1_L1"),
      Map.entry(AutoChooser.AUTOLEAVE.toString( ) + StartPose.START2.toString( ), "Start2_L2"),
      Map.entry(AutoChooser.AUTOLEAVE.toString( ) + StartPose.START3.toString( ), "Start3_L3"),

      Map.entry(AutoChooser.AUTOPRELOAD.toString( ) + StartPose.START1.toString( ), "Start1_RJ"),
      Map.entry(AutoChooser.AUTOPRELOAD.toString( ) + StartPose.START2.toString( ), "Start2_RH"),
      Map.entry(AutoChooser.AUTOPRELOAD.toString( ) + StartPose.START3.toString( ), "Start3_RE"),

      Map.entry(AutoChooser.AUTOPRELOADCORAL.toString( ) + StartPose.START1.toString( ), "Start1_RJ_S1R_RK"),
      Map.entry(AutoChooser.AUTOPRELOADCORAL.toString( ) + StartPose.START2.toString( ), "Start2_RH_S1R_RI"),
      Map.entry(AutoChooser.AUTOPRELOADCORAL.toString( ) + StartPose.START3.toString( ), "Start3_RE_S2L_RC"),

      Map.entry(AutoChooser.AUTOPRELOADCORAL2.toString( ) + StartPose.START1.toString( ), "Start1_RJ_S1R_RK_S1R_RL"),
      Map.entry(AutoChooser.AUTOPRELOADCORAL2.toString( ) + StartPose.START2.toString( ), "Start2_RH_S1R_RK_S1R_RA"),
      Map.entry(AutoChooser.AUTOPRELOADCORAL2.toString( ) + StartPose.START3.toString( ), "Start3_RE_S2L_RD_S2L_RC"),

      Map.entry(AutoChooser.AUTOPRELOADCORAL3.toString( ) + StartPose.START1.toString( ), "Start1_RJ_S1R_RK_S1R_RL_S1R_RA"),
      Map.entry(AutoChooser.AUTOPRELOADCORAL3.toString( ) + StartPose.START2.toString( ), "Start2_RH_S1R_RK_S1R_RA_S1R_RB"),
      Map.entry(AutoChooser.AUTOPRELOADCORAL3.toString( ) + StartPose.START3.toString( ), "Start3_RE_S2L_RD_S2L_RC_S2L_RB"),

      Map.entry(AutoChooser.AUTOPRELOADALGAE.toString( ) + StartPose.START1.toString( ), "Start1_RJ_RIJ_Net"),
      Map.entry(AutoChooser.AUTOPRELOADALGAE.toString( ) + StartPose.START2.toString( ), "Start2_RH_RGH_Net"),
      Map.entry(AutoChooser.AUTOPRELOADALGAE.toString( ) + StartPose.START3.toString( ), "Start3_RE_REF_Proc"),

      Map.entry(AutoChooser.AUTOPRELOADALGAE2.toString( ) + StartPose.START1.toString( ), "Start1_RJ_RIJ_Net_RGH_Net"),
      Map.entry(AutoChooser.AUTOPRELOADALGAE2.toString( ) + StartPose.START2.toString( ), "Start2_RH_RGH_Net_RIJ_Net"),
      Map.entry(AutoChooser.AUTOPRELOADALGAE2.toString( ) + StartPose.START3.toString( ), "Start3_RE_REF_Proc_RGH_Net"),

      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START1.toString( ), "Start1_test1"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START2.toString( ), "Start2_test2"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START3.toString( ), "Start3_test3")));

  /****************************************************************************
   * 
   * The main container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer( )
  {
    Robot.timeMarker("robotContainer: before heading controller and field layout");

    facing.HeadingController = new PhoenixPIDController(kHeadingKp, kHeadingKi, kHeadingKd);    // Swerve steer PID for facing swerve request
    facing.HeadingController.enableContinuousInput(-180.0, 180.0);

    // Identify the field
    DataLogManager.log(String.format("Field: %s width %.2f length %.2f", VIConsts.kGameField.toString( ),
        VIConsts.kATField.getFieldWidth( ), VIConsts.kATField.getFieldLength( )));
    for (int i = 1; i <= 22; i++)
    {
      DataLogManager.log(String.format("Field: ID %d %s", i, VIConsts.kATField.getTagPose(i)));
    }

    addDashboardWidgets( );           // Add some dashboard widgets for commands
    configureButtonBindings( );       // Configure game controller buttons
    initDefaultCommands( );           // Initialize subsystem default commands

    Robot.timeMarker("robotContainer: after default commands");
  }

  /****************************************************************************
   * 
   * Callbacks used by dashboard autonomous choosers to reload when an onChange event occurs
   */
  public void updateAutoChooserCallback(AutoChooser option)
  {
    Robot.reloadAutomousCommand(option.toString( ));
  }

  public void updateStartChooserCallback(StartPose option)
  {
    Robot.reloadAutomousCommand(option.toString( ));
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addDashboardWidgets( )
  {
    // Network tables publisher objects
    m_reefLevelPub =
        NetworkTableInstance.getDefault( ).getTable(Constants.kRobotString).getIntegerTopic(ELConsts.kReefLevelString).publish( );
    m_reefOffsetPub = NetworkTableInstance.getDefault( ).getTable(Constants.kRobotString)
        .getIntegerTopic(VIConsts.kReefOffsetString).publish( );
    m_reefLevelPub.set(3);  // Default to level 3 during auto TODO: make 4 the default!
    m_reefOffsetPub.set(0); // Default to left branch

    // Build autonomous chooser objects on dashboard and fill the options
    SmartDashboard.putData("AutoMode", m_autoChooser);
    SmartDashboard.putData("StartPosition", m_startChooser);
    SmartDashboard.putNumber("AutoDelay", 0.0);

    // Configure autonomous sendable chooser
    m_autoChooser.setDefaultOption("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.addOption("1 - AutoLeave", AutoChooser.AUTOLEAVE);
    m_autoChooser.addOption("2 - AutoPreload", AutoChooser.AUTOPRELOAD);
    m_autoChooser.addOption("3 - AutoPreloadCoral", AutoChooser.AUTOPRELOADCORAL);
    m_autoChooser.addOption("4 - AutoPreloadCoral2", AutoChooser.AUTOPRELOADCORAL2);
    m_autoChooser.addOption("5 - AutoPreloadCoral3", AutoChooser.AUTOPRELOADCORAL3);
    m_autoChooser.addOption("6 - AutoPreloadAlgae", AutoChooser.AUTOPRELOADALGAE);
    m_autoChooser.addOption("7 - AutoPreloadAlgae2", AutoChooser.AUTOPRELOADALGAE2);
    m_autoChooser.addOption("8 - AutoTestPath", AutoChooser.AUTOTEST);
    m_autoChooser.onChange(this::updateAutoChooserCallback);

    // Configure starting pose sendable chooser
    m_startChooser.setDefaultOption("START1", StartPose.START1);
    m_startChooser.addOption("START2", StartPose.START2);
    m_startChooser.addOption("START3", StartPose.START3);
    m_startChooser.onChange(this::updateStartChooserCallback);

    // Add a button that allows running autonomous commands in teleop
    SmartDashboard.putData("AutoChooserRun", new InstantCommand(( ) ->
    {
      if (m_autoCommand.isScheduled( ))
      {
        m_autoCommand.cancel( );
      }
      if ((m_autoCommand = getAutonomousCommand( )) != null)
      {
        m_autoCommand.schedule( );
      }
    }));

    // Add buttons for testing HID rumble features to dashboard
    Time duration = Seconds.of(1.0);
    SmartDashboard.putData("HIDRumbleDriver",
        m_hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, duration, Constants.kRumbleIntensity));
    SmartDashboard.putData("HIDRumbleOperator",
        m_hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, duration, Constants.kRumbleIntensity));

    // Add subsystem command objects and main scheduler to dashboard
    SmartDashboard.putData("elevator", m_elevator);
    SmartDashboard.putData("manipulator", m_manipulator);

    SmartDashboard.putData(CommandScheduler.getInstance( ));

    // Add command groups to dashboard
    SmartDashboard.putData("AcquireAlgae", new AcquireAlgae(m_elevator, m_manipulator, m_led, m_hid));
    SmartDashboard.putData("AcquireCoral", new AcquireCoral(m_elevator, m_manipulator, m_led, m_hid));
    SmartDashboard.putData("ScoreAlgae", new ScoreAlgae(m_elevator, m_manipulator, m_led, m_hid));
    SmartDashboard.putData("ScoreCoral", new ScoreCoral(m_elevator, m_manipulator, m_led, m_hid));
  }

  /****************************************************************************
   * 
   * Define button-command mappings. Triggers are created and bound to the desired commands.
   */
  private void configureButtonBindings( )
  {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    //
    m_driverPad.a( ).onTrue(new LogCommand("driverPad", "A"));
    m_driverPad.b( ).whileTrue(new DeferredCommand(( ) -> m_drivetrain.getReefAlignmentCommand( ), Set.of(m_drivetrain)));
    m_driverPad.x( ).onTrue(new LogCommand("driverPad", "X"));
    m_driverPad.y( ).whileTrue(getSlowSwerveCommand( )); // Note: left lower paddle!

    //
    // Driver - Bumpers, start, back
    //
    m_driverPad.leftBumper( ).onTrue(new AcquireAlgae(m_elevator, m_manipulator, m_led, m_hid));
    m_driverPad.rightBumper( ).onTrue(new AcquireCoral(m_elevator, m_manipulator, m_led, m_hid));
    m_driverPad.back( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));                             // aka View button
    m_driverPad.start( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldCentric( )));         // aka Menu button

    //
    // Driver - POV buttons
    //
    m_driverPad.pov(0).whileTrue(m_drivetrain.applyRequest(( ) -> facing    //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(0.0))));
    m_driverPad.pov(45).whileTrue(m_drivetrain.applyRequest(( ) -> facing   //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(-60.0))));
    m_driverPad.pov(90).whileTrue(m_drivetrain.applyRequest(( ) -> facing   //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(-90.0))));
    m_driverPad.pov(135).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(-120.0))));
    m_driverPad.pov(180).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(-180.0))));
    m_driverPad.pov(225).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(120.0))));
    m_driverPad.pov(270).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(90.0))));
    m_driverPad.pov(315).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(60.0))));

    //
    // Driver Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_driverPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new ScoreAlgae(m_elevator, m_manipulator, m_led, m_hid));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ScoreCoral(m_elevator, m_manipulator, m_led, m_hid));

    m_driverPad.leftStick( ).onTrue(new LogCommand("driverPad", "left stick"));
    m_driverPad.rightStick( ).onTrue(new LogCommand("driverPad", "right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    //
    m_operatorPad.a( ).onTrue(new LogCommand("operPad", "A"));
    m_operatorPad.b( ).onTrue(getReefOffsetSelectCommand(1));
    m_operatorPad.x( ).onTrue(getReefOffsetSelectCommand(0));
    m_operatorPad.y( ).onTrue(getReefOffsetSelectCommand(2));

    //
    // Operator - Bumpers, start, back
    //
    m_operatorPad.leftBumper( ).onTrue(new AcquireAlgae(m_elevator, m_manipulator, m_led, m_hid));
    m_operatorPad.rightBumper( ).onTrue(new AcquireCoral(m_elevator, m_manipulator, m_led, m_hid));
    m_operatorPad.back( ).toggleOnTrue(m_elevator.getJoystickCommand(( ) -> getElevatorAxis( )));   // aka View button
    m_operatorPad.start( ).toggleOnTrue(m_manipulator.getJoystickCommand(( ) -> getWristAxis( )));  // aka Menu button

    //
    // Operator - POV buttons
    //
    m_operatorPad.pov(0).onTrue(getReefLevelSelectCommand(4));
    m_operatorPad.pov(90).onTrue(getReefLevelSelectCommand(1));
    m_operatorPad.pov(180).onTrue(getReefLevelSelectCommand(2));
    m_operatorPad.pov(270).onTrue(getReefLevelSelectCommand(3));

    //
    // Operator Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new ScoreAlgae(m_elevator, m_manipulator, m_led, m_hid));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ScoreCoral(m_elevator, m_manipulator, m_led, m_hid));

    m_operatorPad.leftStick( ).toggleOnTrue(new LogCommand("operPad", "left stick"));
    m_operatorPad.rightStick( ).toggleOnTrue(new LogCommand("operPad", "right stick"));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    if (!m_macOSXSim)
    {
      m_drivetrain.setDefaultCommand(                                                         // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                              //
              .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                       // Drive forward with negative Y (forward)
              .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                       // Drive left with negative X (left)
              .withRotationalRate(kMaxAngularRate.times(-m_driverPad.getRightX( )))           // Drive counterclockwise with negative X (left)
          )                                                                                   //
              .withName("CommandSwerveDrivetrain"));
    }
    else // When using simulation on MacOS X, XBox controllers need to be re-mapped due to an Apple bug
    {
      m_drivetrain.setDefaultCommand(                                                         // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                              //
              .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                       // Drive forward with negative Y (forward)
              .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                       // Drive left with negative X (left)
              .withRotationalRate(kMaxAngularRate.times(-m_driverPad.getLeftTriggerAxis( )))  // Drive counterclockwise with negative X (left)
          )                                                                                   //
              .withName("CommandSwerveDrivetrain"));
    }

    m_drivetrain.registerTelemetry(logger::telemeterize);

    // Note: Only one default command can be active per subsystem--use the manual modes during bring-up

    // Default command - Motion Magic hold
    m_elevator.setDefaultCommand(m_elevator.getHoldPositionCommand(m_elevator::getCurrentHeight));
    m_manipulator.setDefaultCommand(m_manipulator.getHoldPositionCommand(ClawMode.CORALMAINTAIN, m_manipulator::getCurrentAngle));

    // Default command - manual mode (use these during robot bringup)
    // m_elevator.setDefaultCommand(m_elevator.getJoystickCommand(( ) -> getElevatorAxis( )));
    // m_manipulator.setDefaultCommand(m_manipulator.getJoystickCommand(( ) -> getWristAxis( )));
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
    List<PathPlannerPath> ppPathList;
    try
    {
      ppPathList = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    }
    catch (ParseException | IOException e)
    {
      DataLogManager.log(String.format("getAuto: ERROR - parse or IO exception when reading the auto file"));
      return m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
    }

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

    // {
    //   // Debug only: print states of first path
    //   List<PathPlannerTrajectory.State> states = initialPath.getTrajectory(new ChassisSpeeds( ), new Rotation2d( )).getStates( );
    //   for (int i = 0; i < states.size( ); i++)
    //     DataLogManager.log(String.format("autoCommand: Auto path state: (%d) %s", i, states.get(i).getTargetHolonomicPose( )));
    // }

    // Set field centric robot position to start of auto sequence
    Optional<Pose2d> startPose;
    try
    {
      startPose = initialPath.getStartingHolonomicPose( );
    }
    catch (Exception nullException)
    {
      DataLogManager.log(String.format("getAuto: starting pose is missing"));
      return m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
    }
    m_drivetrain.resetPose(startPose.get( ));

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
      case AUTOPRELOAD :
        m_autoCommand =
            new AutoPreload(ppPathList, m_drivetrain, m_elevator, m_manipulator, m_led, m_hid, this::getReefLevelCommand);
        break;
      case AUTOPRELOADCORAL :
        m_autoCommand =
            new AutoPreloadCoral(ppPathList, m_drivetrain, m_elevator, m_manipulator, m_led, m_hid, this::getReefLevelCommand);
        break;
      case AUTOPRELOADCORAL2 :
        m_autoCommand =
            new AutoPreloadCoral2(ppPathList, m_drivetrain, m_elevator, m_manipulator, m_led, m_hid, this::getReefLevelCommand);
        break;
      case AUTOPRELOADCORAL3 :
        m_autoCommand =
            new AutoPreloadCoral3(ppPathList, m_drivetrain, m_elevator, m_manipulator, m_led, m_hid, this::getReefLevelCommand);
        break;
      case AUTOPRELOADALGAE :
        m_autoCommand =
            new AutoPreloadAlgae(ppPathList, m_drivetrain, m_elevator, m_manipulator, m_led, m_hid, this::getReefLevelCommand);
        break;
      case AUTOPRELOADALGAE2 :
        m_autoCommand =
            new AutoPreloadAlgae2(ppPathList, m_drivetrain, m_elevator, m_manipulator, m_led, m_hid, this::getReefLevelCommand);
        break;
      case AUTOTEST :
        m_autoCommand = new AutoTest(ppPathList, m_drivetrain);
        break;
    }

    DataLogManager.log(String.format("getAuto: autoMode %s startOption %s (%s)", autoKey, startPose, m_autoCommand.getName( )));

    double delay = SmartDashboard.getNumber("AutoDelay", 0.0);
    m_autoCommand = new SequentialCommandGroup(                                                       //
        new InstantCommand(( ) -> Robot.timeMarker("AutoStart")),                                 //
        new LogCommand("Autodelay", String.format("Delaying %.1f seconds ...", delay)), //
        new WaitCommand(delay),                                                                       //
        m_autoCommand,                                                                                //
        new InstantCommand(( ) -> Robot.timeMarker("AutoEnd"))                                    //
    );

    return m_autoCommand;
  }

  /****************************************************************************
   * 
   * Use to slow down swerve drivetrain to 30 percent max speed. Drivetrain will execute this command
   * when invoked
   */
  public Command getSlowSwerveCommand( )
  {
    return m_drivetrain.applyRequest(( ) -> drive                                                 // 
        .withVelocityX(kMaxSpeed.times(kSlowSwerve).times(-m_driverPad.getLeftY( )))              // Drive forward with negative Y (forward)
        .withVelocityY(kMaxSpeed.times(kSlowSwerve).times(-m_driverPad.getLeftX( )))              // Drive left with negative X (left)
        .withRotationalRate(kMaxAngularRate.times(kSlowSwerve).times(-m_driverPad.getRightX( )))                     // Drive counterclockwise with negative X (left)
    )                                                                                             //
        .withName("CommandSlowSwerveDrivetrain");
  }

  /****************************************************************************
   * 
   * Create Reef Level Select Command
   * 
   * @return instant command to Select Reef Scoring Level
   */
  private Command getReefLevelSelectCommand(int level)
  {
    return new InstantCommand(          // Command with init only phase declared
        ( ) ->
        {
          m_reefLevelPub.set(level);
        }).withName(ELConsts.kReefLevelString).ignoringDisable(true);
  }

  public Command getReefLevelCommand( ) // Command supplier to set default reef level
  {
    return getReefLevelSelectCommand(3);
  }

  /****************************************************************************
   * 
   * Create Reef Branch Select Command
   * 
   * @return instant command to Select Reef Branch
   */

  private Command getReefOffsetSelectCommand(int branch)
  {
    return new InstantCommand(          // Command with init only phase declared
        ( ) ->
        {
          m_reefOffsetPub.set(branch);
        }).withName(VIConsts.kReefOffsetString).ignoringDisable(true);
  }

  /****************************************************************************
   * 
   * Create Select Branch Command
   * 
   * @return instant command to Select Branch Alignment
   */
  public Command getAlignToReefCommand( )
  {
    return (m_drivetrain.getReefAlignmentCommand( ));
  }

  /****************************************************************************
   * 
   * Gamepad joystick axis interfaces
   */
  public double getElevatorAxis( )
  {
    return -m_operatorPad.getLeftY( );
  }

  /****************************************************************************
   * 
   * Gamepad joystick axis interfaces
   */
  public double getWristAxis( )
  {
    return m_operatorPad.getRightX( );
  }

  /****************************************************************************
   * 
   * Called by disabledInit - place subsystem initializations here
   */
  public void disabledInit( )
  {
    m_led.initialize( );
    m_power.initialize( );
    m_vision.initialize( );

    m_elevator.initialize( );
    m_manipulator.initialize( );

    m_vision.SetCPUThrottleLevel(false);
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void autoInit( )
  {
    m_vision.SetCPUThrottleLevel(true);
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void teleopInit( )
  {
    m_vision.SetCPUThrottleLevel(true);
  }

  /****************************************************************************
   * 
   * Called when user button is pressed - place subsystem fault dumps here
   */
  public void printFaults( )
  {
    m_led.printFaults( );
    m_power.printFaults( );

    m_elevator.printFaults( );
    m_manipulator.printFaults( );
  }
}
