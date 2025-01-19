//
// Power subystem - handles Power Distribution Hub readings
//
package frc.robot.subsystems;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/****************************************************************************
 * 
 * Power subsystem class
 */
public class Power extends SubsystemBase
{
  // Member objects
  private static final PowerDistribution m_powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  /****************************************************************************
   * 
   * Constructor
   */
  public Power( )
  {
    setName("Power");
    setSubsystem("Power");

    addChild("PowerDistribution", m_powerDistribution);

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

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    DataLogManager
        .log(String.format("%s: Subsystem initialized!  %.1f volts", getSubsystem( ), m_powerDistribution.getVoltage( )));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", getSubsystem( )));
    DataLogManager.log(String.format("  Temperature ...... %.1f C", m_powerDistribution.getTemperature( )));
    DataLogManager.log(String.format("  Input Voltage .... %.1f volts", m_powerDistribution.getVoltage( )));
    for (int i = 0; i < m_powerDistribution.getNumChannels( ); i++)
    {
      DataLogManager.log(String.format("  Channel %2d ....... %.1f amps", i, m_powerDistribution.getCurrent(i)));
    }
    DataLogManager.log(String.format("  Total Current .... %.1f Amps", m_powerDistribution.getTotalCurrent( )));
    DataLogManager.log(String.format("  Total Power ...... %.1f watts", m_powerDistribution.getTotalPower( )));
    DataLogManager.log(String.format("  Total Energy ..... %.1f joules", m_powerDistribution.getTotalEnergy( )));

    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", getSubsystem( )));
    PowerDistributionFaults faults = m_powerDistribution.getFaults( );
    PowerDistributionStickyFaults stickyFaults = m_powerDistribution.getStickyFaults( );
    DataLogManager.log(String.format("  Brownout ......... %5s %5s", faults.Brownout, stickyFaults.Brownout));
    DataLogManager.log(String.format("  CanBusOff ........ %5s %5s", "", stickyFaults.CanBusOff));
    DataLogManager.log(String.format("  CanWarning ....... %5s %5s", faults.CanWarning, stickyFaults.CanWarning));
    DataLogManager.log(String.format("  HardwareFault .... %5s %5s", faults.HardwareFault, ""));
    DataLogManager.log(String.format("  HasReset ......... %5s %5s", "", stickyFaults.HasReset));
    DataLogManager
        .log(String.format("  Channel0BreakerFault ... %5s %5s", faults.Channel0BreakerFault, stickyFaults.Channel0BreakerFault));
    DataLogManager
        .log(String.format("  Channel1BreakerFault ... %5s %5s", faults.Channel1BreakerFault, stickyFaults.Channel1BreakerFault));
    DataLogManager
        .log(String.format("  Channel2BreakerFault ... %5s %5s", faults.Channel2BreakerFault, stickyFaults.Channel2BreakerFault));
    DataLogManager
        .log(String.format("  Channel3BreakerFault ... %5s %5s", faults.Channel3BreakerFault, stickyFaults.Channel3BreakerFault));
    DataLogManager
        .log(String.format("  Channel4BreakerFault ... %5s %5s", faults.Channel4BreakerFault, stickyFaults.Channel4BreakerFault));
    DataLogManager
        .log(String.format("  Channel5BreakerFault ... %5s %5s", faults.Channel5BreakerFault, stickyFaults.Channel5BreakerFault));
    DataLogManager
        .log(String.format("  Channel6BreakerFault ... %5s %5s", faults.Channel6BreakerFault, stickyFaults.Channel6BreakerFault));
    DataLogManager
        .log(String.format("  Channel7BreakerFault ... %5s %5s", faults.Channel7BreakerFault, stickyFaults.Channel7BreakerFault));
    DataLogManager
        .log(String.format("  Channel8BreakerFault ... %5s %5s", faults.Channel8BreakerFault, stickyFaults.Channel8BreakerFault));
    DataLogManager
        .log(String.format("  Channel9BreakerFault ... %5s %5s", faults.Channel9BreakerFault, stickyFaults.Channel9BreakerFault));
    DataLogManager.log(
        String.format("  Channel10BreakerFault .. %5s %5s", faults.Channel10BreakerFault, stickyFaults.Channel10BreakerFault));
    DataLogManager.log(
        String.format("  Channel11BreakerFault .. %5s %5s", faults.Channel11BreakerFault, stickyFaults.Channel11BreakerFault));
    DataLogManager.log(
        String.format("  Channel12BreakerFault .. %5s %5s", faults.Channel12BreakerFault, stickyFaults.Channel12BreakerFault));
    DataLogManager.log(
        String.format("  Channel13BreakerFault .. %5s %5s", faults.Channel13BreakerFault, stickyFaults.Channel13BreakerFault));
    DataLogManager.log(
        String.format("  Channel14BreakerFault .. %5s %5s", faults.Channel14BreakerFault, stickyFaults.Channel14BreakerFault));
    DataLogManager.log(
        String.format("  Channel15BreakerFault .. %5s %5s", faults.Channel15BreakerFault, stickyFaults.Channel15BreakerFault));
    DataLogManager.log(
        String.format("  Channel16BreakerFault .. %5s %5s", faults.Channel16BreakerFault, stickyFaults.Channel16BreakerFault));
    DataLogManager.log(
        String.format("  Channel17BreakerFault .. %5s %5s", faults.Channel17BreakerFault, stickyFaults.Channel17BreakerFault));
    DataLogManager.log(
        String.format("  Channel18BreakerFault .. %5s %5s", faults.Channel18BreakerFault, stickyFaults.Channel18BreakerFault));
    DataLogManager.log(
        String.format("  Channel19BreakerFault .. %5s %5s", faults.Channel19BreakerFault, stickyFaults.Channel19BreakerFault));
    DataLogManager.log(
        String.format("  Channel20BreakerFault .. %5s %5s", faults.Channel20BreakerFault, stickyFaults.Channel20BreakerFault));
    DataLogManager.log(
        String.format("  Channel21BreakerFault .. %5s %5s", faults.Channel21BreakerFault, stickyFaults.Channel21BreakerFault));
    DataLogManager.log(
        String.format("  Channel22BreakerFault .. %5s %5s", faults.Channel22BreakerFault, stickyFaults.Channel22BreakerFault));
    DataLogManager.log(
        String.format("  Channel23BreakerFault .. %5s %5s", faults.Channel23BreakerFault, stickyFaults.Channel23BreakerFault));

    // m_powerDistribution.getFaults().
    m_powerDistribution.resetTotalEnergy( );
    m_powerDistribution.clearStickyFaults( );
  }
}
