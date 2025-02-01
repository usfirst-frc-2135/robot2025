package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ExampleSmartMotorController.PIDMode;

public class Robot extends TimedRobot
{
  private static final double kEncoderCPR = 4096;

  private static enum ControlMode
  {
    kStopped, kFixedSpeed, kJoystickControl, kClosedLoop
  }

  private final static XboxController               m_controller    = new XboxController(0);
  private final static ExampleSmartMotorController  m_motor1        = new ExampleSmartMotorController(5, kEncoderCPR);
  private final static ExampleSmartMotorController  m_motor2        = new ExampleSmartMotorController(6, kEncoderCPR);

  private final static TalonSRXSimCollection        m_motor1Sim     = m_motor1.getMotorSimulation( );
  private final static ElevSim                      m_elevSim       = new ElevSim(m_motor1Sim, kEncoderCPR);

  private final static double                       kv              = 1.0;    // Max velocity - RPS
  private final static double                       ka              = 2.0;    // Max acceleration - RPS^2

  private final static TrapezoidProfile.Constraints m_constraints   = new TrapezoidProfile.Constraints(kv, ka);

  private final static double                       m_goal1         = 0.5;    // Goal 1 position
  private final static double                       m_goal2         = -1.0;   // Goal 2 position

  private ControlMode                               m_controlMode   = ControlMode.kStopped;
  private double                                    m_fixedSpeed    = 0.3;
  private double                                    m_percentOutput = 0.0;

  private Timer                                     m_timer         = new Timer( );
  private TrapezoidProfile                          m_profile       = new TrapezoidProfile(m_constraints);
  private TrapezoidProfile.State                    m_goal          = new TrapezoidProfile.State( );
  private TrapezoidProfile.State                    m_setpoint      = new TrapezoidProfile.State( );

  /**
   * robotInit - called ONCE when the robot class starts up
   */
  @Override
  public void robotInit( )
  {
    DataLogManager.start( );
    SmartDashboard.putNumber("FixedSpeed", m_fixedSpeed);
    DataLogManager.log(String.format("***** Prototype2024 starting up *****"));
  }

  /**
   * robotPeriodic - periodic processing for this motor/controller called every 20 msec
   */
  @Override
  public void robotPeriodic( )
  {
    m_motor1.periodic( );
    m_motor2.periodic( );

    m_elevSim.periodic( );
  }

  /**
   * disabledInit - called ONCE whenever the robot is disabled
   */
  @Override
  public void disabledInit( )
  {
    m_controlMode = ControlMode.kStopped;
    m_goal.position = 0.0;
    m_goal.velocity = 0.0;
    m_setpoint.position = 0.0;
    m_setpoint.velocity = 0.0;
    m_motor1.set(0.0);
    m_motor2.set(0.0);
    m_motor1.resetEncoder( );
    m_motor2.resetEncoder( );
    m_elevSim.reset( );
  }

  /**
   * teleopPeriodic - WHEN ENABLED: periodic processing for this motor/controller called every 20 msec
   * 
   * Button definitions
   * A, B - sets the motor to a constant percentOutput value (runs continuously)
   * X, Y - sets the motor to a positional PID with a pre-determined setpoint
   * Left bumper - controls the motor based on the left Y joystick input
   * Right bumper - sets the motor to a stopped condition
   */
  @Override
  public void teleopPeriodic( )
  {
    // Detect mode changes

    if (m_controller.getAButtonPressed( ) || m_controller.getBButtonPressed( ))
    {
      m_controlMode = ControlMode.kFixedSpeed;
      Boolean aButton = m_controller.getAButton( );
      m_fixedSpeed = SmartDashboard.getNumber("FixedSpeed", m_fixedSpeed);
      DataLogManager.log(String.format("%s button pressed - percent output: %.2f", (aButton) ? "A" : "B", m_fixedSpeed));
      m_percentOutput = (aButton) ? m_fixedSpeed : -m_fixedSpeed;
    }
    else if (m_controller.getRightBumperPressed( ))
    {
      m_controlMode = ControlMode.kStopped;
      DataLogManager.log(String.format("Left bumper pressed - STOP!"));
      m_percentOutput = 0.0;
      m_motor1.resetEncoder( );
      m_motor2.resetEncoder( );
      m_elevSim.reset( );
    }
    else if (m_controller.getLeftBumperPressed( ))
    {
      m_controlMode = ControlMode.kJoystickControl;
      DataLogManager.log(String.format("Right bumper pressed - joystick control"));
    }
    else if (m_controller.getXButtonPressed( ) || m_controller.getYButtonPressed( ))
    {
      m_controlMode = ControlMode.kClosedLoop;
      Boolean xButton = m_controller.getXButton( );
      DataLogManager.log(String.format("%s button pressed", (xButton ? "X" : "Y")));
      m_timer.restart( );
      m_setpoint = new TrapezoidProfile.State( );
      m_goal = new TrapezoidProfile.State(((xButton) ? m_goal1 : m_goal2), 0);
      DataLogManager.log(String.format("Start: goal: %.2f setpoint: %.2f Position: %.3f", m_goal.position, m_setpoint.position,
          m_motor1.getEncoderPosition( )));
    }

    // Run the correct mode each loop

    switch (m_controlMode)
    {
      case kJoystickControl :
        m_percentOutput = MathUtil.applyDeadband(m_controller.getLeftY( ), 0.15);
      default :
      case kStopped :
      case kFixedSpeed :
        m_motor1.set(m_percentOutput);
        m_motor2.set(-m_percentOutput);
        break;

      case kClosedLoop :
      {
        double position = m_motor1.getEncoderPosition( );
        DataLogManager.log(
            String.format("Loop:  goal: %.2f setpoint: %.2f Position: %.3f", m_goal.position, m_setpoint.position, position));
        if (m_timer.hasElapsed(4.0) || Math.abs(position - m_goal.position) < 0.05)
        {
          DataLogManager.log(String.format("Return to open loop - %s", (m_timer.hasElapsed(4.0) ? "timed out" : "at goal")));
          m_controlMode = ControlMode.kStopped;
          m_percentOutput = 0.0;
        }
        else
        {
          DataLogManager.log(
              String.format("Loop:  goal: %.2f setpoint: %.2f Position: %.3f", m_goal.position, m_setpoint.position, position));
          m_setpoint = m_profile.calculate(m_timer.get( ), m_setpoint, m_goal);
          m_motor1.setSetpoint(PIDMode.kPosition, m_setpoint.position, 0.0);
          m_motor2.set(0.0);
        }
      }
        break;
    }

    SmartDashboard.putNumber("LOOP-goal", m_goal.position);
    SmartDashboard.putNumber("LOOP-m_motor1", m_motor1.get( ));
    SmartDashboard.putNumber("LOOP-m_motor2", m_motor2.get( ));
  }

}
