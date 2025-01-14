
package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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
  private static final boolean m_isComp         = detectRobot( );         // Detect which robot is in use
  private final RobotContainer m_robotContainer = new RobotContainer( );  // Create that robot
  private Command              m_autonomousCommand;
  private boolean              m_faultsCleared  = false;
  private static double        m_timeMark       = Timer.getFPGATimestamp( );

  /****************************************************************************
   * 
   * This function runs when the Robot class is first started and used for initialization.
   */
  public Robot( )
  {
    // Starts recording to data log
    DataLogManager.start( );
    DataLogManager.logConsoleOutput(true);
    DriverStation.startDataLog(DataLogManager.getLog( )); // Logs joystick data
    SignalLogger.start( );                              // Start CTRE signal logging
    Robot.timeMarker("Robot: start");

    // Start the web server for remoote dashboard layout
    WebServer.start(5800, Filesystem.getDeployDirectory( ).getPath( ));

    // Log when commands initialize, interrupt, and end states
    CommandScheduler.getInstance( ).onCommandInitialize(cmd -> DataLogManager.log(String.format("%s: Init", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandInterrupt(cmd -> DataLogManager.log(String.format("%s: Interrupt", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandFinish(cmd -> DataLogManager.log(String.format("%s: End", cmd.getName( ))));

    // Forward packets from RoboRIO USB connections to ethernet
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);

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
    SmartDashboard.putNumber("MatchTime", Timer.getMatchTime());
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

    m_robotContainer.initialize( );

    Robot.timeMarker("disabledInit: after init");
  }

  /**
   * This function is called periodically while in Disabled mode.
   */
  @Override
  public void disabledPeriodic( )
  {
    // If RoboRIO User button is pressed, dump all CAN faults
    if (RobotController.getUserButton( ))
    {
      if (!m_faultsCleared)
      {
        m_faultsCleared = true;
        m_robotContainer.printFaults( );
      }
    }
    else
      m_faultsCleared = false;
  }

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit( )
  {
    datalogMatchBanner("autonomousInit");

    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel( );
      m_autonomousCommand = null;
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand( );

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
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel( );
      m_autonomousCommand = null;
    }

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
      robotName = "SIMULATION";
    else if (serialNum.equals(Constants.kCompSN))
    {
      isComp = true;
      robotName = "COMPETITION (A)";
    }
    else if (serialNum.equals(Constants.kBetaSN))
    {
      isComp = false;
      robotName = "PRACTICE/BETA (B)";
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
  public static void datalogMatchBanner(String msg)
  {
    DataLogManager.log(String.format("===================================================================="));
    DataLogManager.log(String.format("%s: Match %s %s, %s Alliance", msg, DriverStation.getMatchType( ).toString( ),
        DriverStation.getMatchNumber( ), DriverStation.getAlliance( ).toString( )));
    DataLogManager.log(String.format("===================================================================="));
  }

}
