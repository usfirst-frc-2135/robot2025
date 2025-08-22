
package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot
{
  private static final boolean m_isComp          = detectRobot( );         // Detect which robot is in use
  private final RobotContainer m_robotContainer  = new RobotContainer( );  // Create that robot
  private Command              m_autonomousCommand;
  private boolean              m_faultsCleared   = false;
  private static double        m_timeMark        = Timer.getFPGATimestamp( );
  private static boolean       m_loadAutoCommand = true;
  private double               m_autoDelay       = 0.0;
  private Optional<Alliance>   m_alliance;

  /****************************************************************************
   * 
   * This function runs when the Robot class is first started and used for initialization.
   */
  public Robot( )
  {
    // Starts recording to data log
    DataLogManager.start( );
    DriverStation.startDataLog(DataLogManager.getLog( )); // Logs joystick data
    Robot.timeMarker("Robot: start");

    // Start the web server for remoote dashboard layout
    WebServer.start(5800, Filesystem.getDeployDirectory( ).getPath( ));

    // Log when commands initialize, interrupt, and end states
    CommandScheduler.getInstance( ).onCommandInitialize(cmd -> DataLogManager.log(String.format("%s: Init", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandInterrupt(cmd -> DataLogManager.log(String.format("%s: Interrupt", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandFinish(cmd -> DataLogManager.log(String.format("%s: End", cmd.getName( ))));

    // Forward packets from RoboRIO USB connections to ethernet
    for (int port = 5800; port <= 5809; port++)
    {
      PortForwarder.add(port, Constants.kLLLeftName + ".local", port);
      PortForwarder.add(port, Constants.kLLRightName + ".local", port);
    }

    FollowPathCommand.warmupCommand( ).withName("PathPlanner - warmupCommand").schedule( ); // Recommended by PathPlanner docs

    Robot.timeMarker("Robot: after warmup");
  }

  /****************************************************************************
   * 
   * This function is called every 20 msec robot loop, no matter the mode. Use this for items like
   * diagnostics that you want to run during Disabled, Autonomous, Teleoperated and Test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic( )
  {
    // Runs the Command Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    //
    CommandScheduler.getInstance( ).run( );
  }

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit( )
  {
    datalogMatchBanner("disabledInit");

    Robot.timeMarker("disabledInit: before init");

    m_robotContainer.disabledInit( );

    Robot.timeMarker("disabledInit: after init");
  }

  /**
   * This function is called periodically while in Disabled mode.
   */
  @Override
  public void disabledPeriodic( )
  {
    double autoDelay = SmartDashboard.getNumber("AutoDelay", 0.0);
    if (autoDelay != m_autoDelay)
    {
      m_loadAutoCommand = true;
      m_autoDelay = autoDelay;
    }

    Optional<Alliance> alliance = DriverStation.getAlliance( );
    if (m_alliance != alliance)
    {
      m_loadAutoCommand = true;
      m_alliance = alliance;
    }

    if (m_loadAutoCommand)
    {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand( );
      m_loadAutoCommand = false;  // Load only once per request
    }

    // If RoboRIO User button is pressed, dump all CAN faults
    if (RobotController.getUserButton( ))
    {
      if (!m_faultsCleared)
      {
        m_faultsCleared = true;
        m_robotContainer.printAllFaults( );
      }
    }
    else
    {
      m_faultsCleared = false;
    }
  }

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit( )
  {
    datalogMatchBanner("autonomousInit");

    cancelOldAutonomousCommand( );

    // Handle any commands that need to be scheduled when entering Teleop mode
    m_robotContainer.autoInit( );

    // schedule the autonomous command selected by the RobotContainer class
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule( );
    }
  }

  /**
   * This function is called periodically during Autonomous mode.
   */
  @Override
  public void autonomousPeriodic( )
  {}

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit( )
  {
    datalogMatchBanner("teleopInit");

    // Make sure that the autonomous command stops running when Teleop starts running
    cancelOldAutonomousCommand( );

    // Handle any commands that need to be scheduled when entering Teleop mode
    m_robotContainer.teleopInit( );
  }

  /**
   * This function is called periodically during Teleop mode (operator control).
   */
  @Override
  public void teleopPeriodic( )
  {}

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Simulation mode.
   */
  @Override
  public void simulationInit( )
  {}

  /**
   * This function is called periodically during Simulation mode.
   */
  @Override
  public void simulationPeriodic( )
  {}

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit( )
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance( ).cancelAll( );
  }

  /**
   * This function is called periodically during Test mode.
   */
  @Override
  public void testPeriodic( )
  {}

  /****************************************************************************
   * 
   * Our robot detection process to differentiate between competition and beta (practice) robots
   */
  public static boolean isComp( )
  {
    return m_isComp;
  }

  private static boolean detectRobot( )
  {
    // Detect which robot/RoboRIO
    String serialNum = System.getenv("serialnum");
    String robotName = "UNKNOWN";
    boolean isComp = false;

    DataLogManager.log(String.format("robotContainer: RoboRIO SN: %s", serialNum));
    if (serialNum == null)
    {
      robotName = "SIMULATION";
    }
    else if (serialNum.equals(Constants.kCompSN))
    {
      isComp = true;
      robotName = "COMPETITION (A)";
    }
    else if (serialNum.equals(Constants.kPracticeSN))
    {
      isComp = false;
      robotName = "PRACTICE (B)";
    }
    DataLogManager.log(String.format("robotContainer: Detected the %s robot (RoboRIO)!", robotName));

    return isComp;
  }

  /****************************************************************************
   * 
   * Method for printing the absolute and relative time from the previous call
   */
  public static void timeMarker(String msg)
  {
    double now = Timer.getFPGATimestamp( );

    DataLogManager.log(String.format("***** TimeMarker ***** absolute: %.3f relative: %.3f - %s", now, now - m_timeMark, msg));

    m_timeMark = now;
  }

  /****************************************************************************
   * 
   * Display a mode change banner for the match type and number
   */
  private static void datalogMatchBanner(String msg)
  {
    DataLogManager.log(String.format("========================================================================"));
    DataLogManager.log(String.format("%s: Match %s %s, %s Alliance", msg, DriverStation.getMatchType( ).toString( ),
        DriverStation.getMatchNumber( ), DriverStation.getAlliance( ).toString( )));
    DataLogManager.log(String.format("========================================================================"));
  }

  /****************************************************************************
   * 
   * Cancel any active autonomous commands
   */
  private void cancelOldAutonomousCommand( )
  {
    if (m_autonomousCommand != null && m_autonomousCommand.isScheduled( ))
    {
      m_autonomousCommand.cancel( );
      m_autonomousCommand = null;
    }
  }

  /****************************************************************************
   * 
   * Signal autonomous command to reload
   */
  public static void reloadAutomousCommand(String optionName)
  {
    DataLogManager.log(String.format("Auto change! - %s", optionName));
    m_loadAutoCommand = true;
  }

}
