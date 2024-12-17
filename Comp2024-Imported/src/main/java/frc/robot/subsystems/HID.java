//
// HID subystem - HID feedback on robot
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/****************************************************************************
 * 
 * HID subsystem class to provide rumble command factory
 */
public class HID extends SubsystemBase
{
  // Constants

  // Member objects
  private GenericHID m_driver;
  private GenericHID m_operator;
  private Timer      m_timerDriver   = new Timer( );
  private Timer      m_timerOperator = new Timer( );
  private Time       m_driverDuration;
  private Time       m_operatorDuration;

  /****************************************************************************
   * 
   * Constructor
   */
  public HID(GenericHID driver, GenericHID operator)
  {
    setName("HID");
    setSubsystem("HID");

    m_driver = driver;
    m_operator = operator;

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    if (m_timerDriver.isRunning( ) && m_timerDriver.hasElapsed(m_driverDuration.in(Seconds)))
    {
      m_timerDriver.stop( );
      setHIDRumbleDriver(false, Seconds.of(0), 0);
    }

    if (m_timerOperator.isRunning( ) && m_timerOperator.hasElapsed(m_operatorDuration.in(Seconds)))
    {
      m_timerOperator.stop( );
      setHIDRumbleOperator(false, Seconds.of(0), 0);
    }
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Initialize dashboard widgets
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {}

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set HID driver rumble based on the requested on/off and intensity
   * 
   * @param rumbleOn
   *          request rumble on or off
   * @param intensity
   *          requested rumble strength
   */
  private void setHIDRumbleDriver(boolean rumbleOn, Time duration, double intensity)
  {
    DataLogManager
        .log(String.format("%s: Rumble driver: %s duration %s intensity: %.1f", getName( ), rumbleOn, duration, intensity));

    if (rumbleOn)
    {
      m_timerDriver.restart( );
      m_driverDuration = duration;
    }

    m_driver.setRumble(RumbleType.kBothRumble, (rumbleOn) ? intensity : 0.0);
  }

  /****************************************************************************
   * 
   * Set HID operator rumble based on the requested on/off and intensity
   * 
   * @param rumbleOn
   *          request rumble on or off
   * @param intensity
   *          requested rumble strength
   */
  private void setHIDRumbleOperator(boolean rumbleOn, Time duration, double intensity)
  {
    DataLogManager
        .log(String.format("%s Rumble operator: %s duration %s intensity: %.1f", getName( ), rumbleOn, duration, intensity));

    if (rumbleOn)
    {
      m_timerOperator.restart( );
      m_operatorDuration = duration;
    }

    m_operator.setRumble(RumbleType.kBothRumble, (rumbleOn) ? intensity : 0.0);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create HID set rumble for driver controller command
   * 
   * @param driverRumble
   *          rumble the driver gamepad
   * @param intensity
   *          stength of the rumble [0.0 .. 1.0]
   * @return instant command that rumbles the gamepad
   */
  public Command getHIDRumbleDriverCommand(boolean driverRumble, Time duration, double intensity)
  {

    return new InstantCommand(                                        // Command that runs exactly once
        ( ) -> setHIDRumbleDriver(driverRumble, duration, intensity),  // Method to call
        this                                                          // Command that runs exactly once
    ).withName("HIDRumbleDriver").ignoringDisable(true);
  }

  /****************************************************************************
   * 
   * Create HID set rumble for operator controller command
   * 
   * @param operatorRumble
   *          rumble the operator gamepad
   * @param intensity
   *          stength of the rumble [0.0 .. 1.0]
   * @return instant command that rumbles the gamepad
   */
  public Command getHIDRumbleOperatorCommand(boolean operatorRumble, Time duration, double intensity)
  {

    return new InstantCommand(                                            // Command that runs exactly once
        ( ) -> setHIDRumbleOperator(operatorRumble, duration, intensity),  // Method to call
        this                                                              // Command that runs exactly once
    ).withName("HIDRumbleOperator").ignoringDisable(true);
  }
}
