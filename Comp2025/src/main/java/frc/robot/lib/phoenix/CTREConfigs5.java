
// Phoenix 5 configurations

package frc.robot.lib.phoenix;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/****************************************************************************
 * 
 * CTRE configuration structure for v5 devices
 */
public final class CTREConfigs5
{
  // Grouped by subsystem

  /****************************************************************************
   * 
   * Intake roller SRX attached to rs775pro
   */
  public static TalonSRXConfiguration intakeRollerConfig( )
  {
    TalonSRXConfiguration config = new TalonSRXConfiguration( );

    return config;
  }

  /****************************************************************************
   * 
   * Feeder roller SRX attached to rs775pro
   */
  public static TalonSRXConfiguration feederRollerConfig( )
  {
    TalonSRXConfiguration config = new TalonSRXConfiguration( );

    return config;
  }

}
