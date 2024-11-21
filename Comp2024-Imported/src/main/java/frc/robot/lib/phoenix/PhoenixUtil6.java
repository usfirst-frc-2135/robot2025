//
// Phoenix 6 initialization utilities
//
package frc.robot.lib.phoenix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/****************************************************************************
 * 
 * Phoenix 6 initialization class
 */
public class PhoenixUtil6
{
  private static final int    kRetries    = 5;    // Number of version check attempts
  private static final String kClassName  = "PhoenixUtil6";
  private static final double kCANTimeout = 0.100;

  private static PhoenixUtil6 m_instance  = null;

  /****************************************************************************
   * 
   * Constructor
   */
  PhoenixUtil6( )
  {}

  /****************************************************************************
   * 
   * Retrieve instance of this class (singleton)
   * 
   * @return reference to this (singleton) class
   */
  public static PhoenixUtil6 getInstance( )
  {
    if (m_instance == null)
      m_instance = new PhoenixUtil6( );

    return m_instance;
  }

  /****************************************************************************
   * 
   * Initialize a Talon FX motor and display the result
   * 
   * @param talonFX
   *          reference to an FX controlled motor
   * @param name
   *          descriptive name of the FX controlled motor
   * @param config
   *          the talon FX configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean talonFXInitialize6(TalonFX talonFX, String name, TalonFXConfiguration config)
  {
    final String devType = "talonFX";
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean talonValid = false;

    // Display Talon firmware versions
    deviceID = talonFX.getDeviceID( );

    // This can take multiple attempts before ready
    for (int i = 0; i < kRetries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = talonFX.getVersion( ).waitForUpdate(0.250);
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.100);
    }

    talonValid = (fwvMajor >= Constants.kPhoenix6MajorVersion);

    if (config != null)
      if ((status = talonFX.getConfigurator( ).apply(config, kCANTimeout)) != StatusCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  getConfigurator.apply - %s!", kClassName, deviceID, name,
            devType, status.getDescription( )));

    if ((status = talonFX.setControl(new VoltageOut(0))) != StatusCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  setControl - %s!", kClassName, deviceID, name, devType,
          status.getDescription( )));

    // Configure sensor settings
    if ((status = talonFX.setPosition(0.0)) != StatusCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  setRotorPosition - %s!", kClassName, deviceID, name, devType,
          status.getDescription( )));

    talonFX.setSafetyEnabled(false);

    DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  ver: %d.%d.%d.%d is %s!", kClassName, deviceID, name, devType,
        fwvMajor, fwvMinor, fwvBugfix, fwvBuild, (talonValid) ? "VALID" : "URESPONSIVE"));

    return talonValid;
  }

  /****************************************************************************
   * 
   * Initialize a CANcoder and display the result
   * 
   * @param canCoder
   *          reference to a CANcoder
   * @param name
   *          descriptive name of the CANcoder
   * @param config
   *          the CANcoder configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean canCoderInitialize6(CANcoder canCoder, String name, CANcoderConfiguration config)
  {
    final String devType = "CANcoder";
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean canCoderValid = false;

    // Display Talon firmware versions
    deviceID = canCoder.getDeviceID( );

    // This can take multiple attempts before ready
    for (int i = 0; i < kRetries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = canCoder.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    canCoderValid = (fwvMajor >= Constants.kPhoenix6MajorVersion);

    if (config != null)
      if ((status = canCoder.getConfigurator( ).apply(config, kCANTimeout)) != StatusCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  getConfigurator.apply - %s!", kClassName, deviceID, name,
            devType, status.getDescription( )));

    DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  ver: %d.%d.%d.%d is %s!", kClassName, deviceID, name, devType,
        fwvMajor, fwvMinor, fwvBugfix, fwvBuild, (canCoderValid) ? "VALID" : "URESPONSIVE"));

    return canCoderValid;
  }

  /****************************************************************************
   * 
   * Initialize a Pigeon2 IMU and display the result
   * 
   * @param pigeon2
   *          reference to a pigeon2
   * @param config
   *          the pigeon2 configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean pigeon2Initialize6(Pigeon2 pigeon2, Pigeon2Configuration config)
  {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    int deviceID = 0;
    int fwvMajor = 0;
    int fwvMinor = 0;
    int fwvBugfix = 0;
    int fwvBuild = 0;
    boolean pigeon2Valid = false;

    // Display Talon firmware versions
    deviceID = pigeon2.getDeviceID( );

    Timer.delay(0.25);

    // This can take multiple attempts before ready
    for (int i = 0; i < kRetries && fwvMajor == 0; i++)
    {
      StatusSignal<Integer> statusSignal = pigeon2.getVersion( );
      status = statusSignal.getStatus( );
      if (status.isOK( ))
      {
        fwvMajor = (statusSignal.getValue( ) >> 24) & 0xff;
        fwvMinor = (statusSignal.getValue( ) >> 16) & 0xff;
        fwvBugfix = (statusSignal.getValue( ) >> 8) & 0xff;
        fwvBuild = (statusSignal.getValue( ) >> 0) & 0xff;
      }
      else
        Timer.delay(0.1);
    }

    pigeon2Valid = (fwvMajor >= Constants.kPhoenix6MajorVersion);

    if (config != null)
      if ((status = pigeon2.getConfigurator( ).apply(config, kCANTimeout)) != StatusCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - pigeon2:      getConfigurator.apply - %s!", kClassName, deviceID,
            status.getDescription( )));

    // Configure sensor settings
    if ((status = pigeon2.setYaw(0.0)) != StatusCode.OK)
      DataLogManager
          .log(String.format("%s: ID %2d - pigeon2:      setYaw - %s!", kClassName, deviceID, status.getDescription( )));

    DataLogManager.log(String.format("%s: ID %2d - pigeon2:      ver: %d.%d.%d.%d is %s!", kClassName, deviceID, fwvMajor,
        fwvMinor, fwvBugfix, fwvBuild, (pigeon2Valid) ? "VALID" : "URESPONSIVE"));

    return pigeon2Valid;
  }

  /****************************************************************************
   * 
   * Print fault flags for a talonFX
   * 
   * @param talonFX
   *          reference to a talonFX motor controller
   * @param name
   *          descriptive name of the talonFX
   */
  public void talonFXPrintFaults(TalonFX talonFX, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    DataLogManager.log(String.format("  FaultField ......................... 0x%08x 0x%08x", talonFX.getFaultField( ).getValue( ),
        talonFX.getStickyFaultField( ).getValue( )));
    DataLogManager.log(String.format("  Hardware ........................... %5s %5s", talonFX.getFault_Hardware( ).getValue( ),
        talonFX.getStickyFault_Hardware( ).getValue( )));
    DataLogManager.log(String.format("  ProcTemp ........................... %5s %5s", talonFX.getFault_ProcTemp( ).getValue( ),
        talonFX.getStickyFault_ProcTemp( ).getValue( )));
    DataLogManager.log(String.format("  DeviceTemp ......................... %5s %5s", talonFX.getFault_DeviceTemp( ).getValue( ),
        talonFX.getStickyFault_DeviceTemp( ).getValue( )));
    DataLogManager.log(String.format("  Undervoltage ....................... %5s %5s",
        talonFX.getFault_Undervoltage( ).getValue( ), talonFX.getStickyFault_Undervoltage( ).getValue( )));
    DataLogManager.log(String.format("  BootDuringEnable ................... %5s %5s",
        talonFX.getFault_BootDuringEnable( ).getValue( ), talonFX.getStickyFault_BootDuringEnable( ).getValue( )));
    DataLogManager.log(String.format("  UnlicensedFeatureInUse ............. %5s %5s",
        talonFX.getFault_UnlicensedFeatureInUse( ).getValue( ), talonFX.getStickyFault_UnlicensedFeatureInUse( ).getValue( )));
    DataLogManager.log(String.format("  BridgeBrownout ..................... %5s %5s",
        talonFX.getFault_BridgeBrownout( ).getValue( ), talonFX.getStickyFault_BridgeBrownout( ).getValue( )));
    DataLogManager.log(String.format("  RemoteSensorReset .................. %5s %5s",
        talonFX.getFault_RemoteSensorReset( ).getValue( ), talonFX.getStickyFault_RemoteSensorReset( ).getValue( )));
    DataLogManager.log(String.format("  MissingDifferentialFX .............. %5s %5s",
        talonFX.getFault_MissingDifferentialFX( ).getValue( ), talonFX.getStickyFault_MissingDifferentialFX( ).getValue( )));
    DataLogManager.log(String.format("  RemoteSensorPosOverflow ............ %5s %5s",
        talonFX.getFault_RemoteSensorPosOverflow( ).getValue( ), talonFX.getStickyFault_RemoteSensorPosOverflow( )));
    DataLogManager.log(String.format("  OverSupplyV ........................ %5s %5s",
        talonFX.getFault_OverSupplyV( ).getValue( ), talonFX.getStickyFault_OverSupplyV( ).getValue( )));
    DataLogManager.log(String.format("  UnstableSupplyV .................... %5s %5s",
        talonFX.getFault_UnstableSupplyV( ).getValue( ), talonFX.getStickyFault_UnstableSupplyV( ).getValue( )));
    DataLogManager.log(String.format("  ReverseHardLimit ................... %5s %5s",
        talonFX.getFault_ReverseHardLimit( ).getValue( ), talonFX.getStickyFault_ReverseHardLimit( ).getValue( )));
    DataLogManager.log(String.format("  ForwardHardLimit ................... %5s %5s",
        talonFX.getFault_ForwardHardLimit( ).getValue( ), talonFX.getStickyFault_ForwardHardLimit( ).getValue( )));
    DataLogManager.log(String.format("  ReverseSoftLimit ................... %5s %5s",
        talonFX.getFault_ReverseSoftLimit( ).getValue( ), talonFX.getStickyFault_ReverseSoftLimit( ).getValue( )));
    DataLogManager.log(String.format("  ForwardSoftLimit ................... %5s %5s",
        talonFX.getFault_ForwardSoftLimit( ).getValue( ), talonFX.getStickyFault_ForwardSoftLimit( ).getValue( )));
    DataLogManager.log(String.format("  RemoteSensorDataInvalid ............ %5s %5s",
        talonFX.getFault_RemoteSensorDataInvalid( ).getValue( ), talonFX.getStickyFault_RemoteSensorDataInvalid( )));
    DataLogManager.log(String.format("  FusedSensorOutOfSync ............... %5s %5s",
        talonFX.getFault_FusedSensorOutOfSync( ).getValue( ), talonFX.getStickyFault_FusedSensorOutOfSync( ).getValue( )));
    DataLogManager.log(String.format("  StatorCurrLimit .................... %5s %5s",
        talonFX.getFault_StatorCurrLimit( ).getValue( ), talonFX.getStickyFault_StatorCurrLimit( ).getValue( )));
    DataLogManager.log(String.format("  SupplyCurrLimit .................... %5s %5s",
        talonFX.getFault_SupplyCurrLimit( ).getValue( ), talonFX.getStickyFault_SupplyCurrLimit( ).getValue( )));
    DataLogManager.log(String.format("  UsingFusedCANcoderWhileUnlicensed .. %5s %5s",
        talonFX.getFault_UsingFusedCANcoderWhileUnlicensed( ).getValue( ),
        talonFX.getStickyFault_UsingFusedCANcoderWhileUnlicensed( ).getValue( )));
  }

  /****************************************************************************
   * 
   * Print fault flags for a CANcoder
   * 
   * @param cancoder
   *          reference to a CANcoder
   * @param name
   *          descriptive name of the CANcoder
   */
  public void cancoderPrintFaults(CANcoder cancoder, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    DataLogManager.log(String.format("  FaultField ......................... 0x%08x 0x%08x",
        cancoder.getFaultField( ).getValue( ), cancoder.getStickyFaultField( ).getValue( )));
    DataLogManager.log(String.format("  Hardware ........................... %5s %5s", cancoder.getFault_Hardware( ).getValue( ),
        cancoder.getStickyFault_Hardware( ).getValue( )));
    DataLogManager.log(String.format("  Undervoltage ....................... %5s %5s",
        cancoder.getFault_Undervoltage( ).getValue( ), cancoder.getStickyFault_Undervoltage( ).getValue( )));
    DataLogManager.log(String.format("  BootDuringEnable ................... %5s %5s",
        cancoder.getFault_BootDuringEnable( ).getValue( ), cancoder.getStickyFault_BootDuringEnable( ).getValue( )));
    DataLogManager.log(String.format("  UnlicensedFeatureInUse ............. %5s %5s",
        cancoder.getFault_UnlicensedFeatureInUse( ).getValue( ), cancoder.getStickyFault_UnlicensedFeatureInUse( ).getValue( )));
    DataLogManager.log(String.format("  BadMagnet .......................... %5s %5s", cancoder.getFault_BadMagnet( ).getValue( ),
        cancoder.getStickyFault_BadMagnet( ).getValue( )));
  }
}
