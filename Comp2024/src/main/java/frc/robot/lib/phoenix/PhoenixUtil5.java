//
// Phoenix 5 initialization utilities
//
package frc.robot.lib.phoenix;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/****************************************************************************
 * 
 * Phoenix 5 initialization class
 */
public class PhoenixUtil5
{
  private static final int    kRetries   = 5;    // Number of version check attempts
  private static final String kClassName = "PhoenixUtil5";

  private static PhoenixUtil5 instance   = null;

  /****************************************************************************
   * 
   * Constructor
   */
  PhoenixUtil5( )
  {}

  /****************************************************************************
   * 
   * Retrieve instance of this class (singleton)
   * 
   * @return reference to this (singleton) class
   */
  public static PhoenixUtil5 getInstance( )
  {
    if (instance == null)
      instance = new PhoenixUtil5( );

    return instance;
  }

  /****************************************************************************
   * 
   * Check last error code for a desired Talon SRX amotor and display the result
   * 
   * @param talonSRX
   *          reference to an SRX controlled motor
   * @param message
   *          informational string about the error being checked
   * @return the last error code
   */
  public ErrorCode talonSRXCheckError(WPI_TalonSRX talonSRX, String message)
  {
    ErrorCode errorCode = talonSRX.getLastError( );
    int deviceID = talonSRX.getDeviceID( );

    if (errorCode != ErrorCode.OK)
      DataLogManager.log(String.format("%s: ID %2d - Msg: %s - error %d", kClassName, deviceID, message, errorCode.value));

    return errorCode;
  }

  /****************************************************************************
   * 
   * Initialize a Talon SRX motor and display the result
   * 
   * @param talonSRX
   *          reference to an SRX controlled motor
   * @param name
   *          descriptive name of the SRX controlled motor
   * @param config
   *          the talon SRX configuration to be programmed
   * @return true if successfully initialized
   */
  public boolean talonSRXInitialize(WPI_TalonSRX talonSRX, String name, TalonSRXConfiguration config)
  {
    final String devType = "talonSRX";
    ErrorCode error = ErrorCode.OK;
    int deviceID = 0;
    int fwVersion = 0;
    String verStr = "ver: unknown";
    boolean talonSRXValid = false;
    boolean initialized = false;

    // Display Talon firmware versions
    deviceID = talonSRX.getDeviceID( );
    error = talonSRXCheckError(talonSRX, String.format(" %15s %8s:  getDeviceID error", name, devType));

    Timer.delay(0.250);

    if (error == ErrorCode.OK)
    {
      // This can take multiple attempts before ready
      for (int i = 0; i < kRetries && fwVersion == 0; i++)
      {
        fwVersion = talonSRX.getFirmwareVersion( );
        error = talonSRXCheckError(talonSRX, String.format(" %15s %8s:  getFirmwareVersion error", name, devType));

        if (error == ErrorCode.OK)
        {
          talonSRXValid = true;
          if (fwVersion < Constants.kPhoenix5MajorVersion)
            DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  Incorrect FW version: %d - error %d", kClassName, deviceID,
                name, devType, (fwVersion / 256), error.value));
          break;
        }

        Timer.delay(0.100);
      }
    }

    if (talonSRXValid)
    {
      verStr = "ver: " + (fwVersion / 256.0);
      error = talonSRX.configFactoryDefault( );
      if (error != ErrorCode.OK)
        DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  %s Msg: configFactoryDefault error - %d", kClassName, deviceID,
            name, devType, verStr, error.value));
      else
      {
        initialized = true;
        talonSRX.configAllSettings(config);
        error = talonSRXCheckError(talonSRX, String.format(" %15s %8s:  %s configAllSettings error", name, devType, verStr));
      }
    }

    DataLogManager.log(String.format("%s: ID %2d - %15s %8s:  %s is %s", kClassName, deviceID, name, devType, verStr,
        (talonSRXValid && initialized) ? "VALID!" : "UNRESPONSIVE!"));

    return talonSRXValid && initialized;
  }

  /****************************************************************************
   * 
   * Print fault flags for a talonSRX
   * 
   * @param talonSRX
   *          reference to a talonSRX motor controller
   * @param name
   *          descriptive name of the talonSRX
   */
  public void talonSRXPrintFaults(WPI_TalonSRX talonSRX, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    Faults faults = new Faults( );
    StickyFaults stickyFaults = new StickyFaults( );
    talonSRX.getFaults(faults);
    talonSRX.getStickyFaults(stickyFaults);

    DataLogManager.log(String.format("  APIError ............ %5s %5s", faults.APIError, stickyFaults.APIError));
    DataLogManager
        .log(String.format("  ForwardLimitSwitch .. %5s %5s", faults.ForwardLimitSwitch, stickyFaults.ForwardLimitSwitch));
    DataLogManager.log(String.format("  ForwardSoftLimit .... %5s %5s", faults.ForwardSoftLimit, stickyFaults.ForwardSoftLimit));
    DataLogManager.log(String.format("  HardwareESDReset .... %5s %5s", faults.HardwareESDReset, stickyFaults.HardwareESDReset));
    DataLogManager.log(String.format("  HardwareFailure ..... %5s %5s", faults.HardwareFailure, ""));
    DataLogManager
        .log(String.format("  RemoteLossOfSignal .. %5s %5s", faults.RemoteLossOfSignal, stickyFaults.RemoteLossOfSignal));
    DataLogManager.log(String.format("  ResetDuringEn ....... %5s %5s", faults.ResetDuringEn, stickyFaults.ResetDuringEn));
    DataLogManager
        .log(String.format("  ReverseLimitSwitch .. %5s %5s", faults.ReverseLimitSwitch, stickyFaults.ReverseLimitSwitch));
    DataLogManager.log(String.format("  ReverseSoftLimit .... %5s %5s", faults.ReverseSoftLimit, stickyFaults.ReverseSoftLimit));
    DataLogManager.log(String.format("  SensorOutOfPhase .... %5s %5s", faults.SensorOutOfPhase, stickyFaults.SensorOutOfPhase));
    DataLogManager.log(String.format("  SensorOverflow ...... %5s %5s", faults.SensorOverflow, stickyFaults.SensorOverflow));
    DataLogManager.log(String.format("  SupplyOverV ......... %5s %5s", faults.SupplyOverV, stickyFaults.SupplyOverV));
    DataLogManager.log(String.format("  SupplyUnstable ...... %5s %5s", faults.SupplyUnstable, stickyFaults.SupplyUnstable));
    DataLogManager.log(String.format("  UnderVoltage ........ %5s %5s", faults.UnderVoltage, stickyFaults.UnderVoltage));
  }

  /****************************************************************************
   * 
   * Print fault flags for a CANdle
   * 
   * @param candle
   *          reference to a CANdle LED controller
   * @param name
   *          descriptive name of the CANdle
   */
  public void candlePrintFaults(CANdle candle, String name)
  {
    DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", name));
    CANdleFaults faults = new CANdleFaults( );
    candle.getFaults(faults);
    // CANdleStickyFaults stickyFaults = new CANdleStickyFaults( );
    // candle.getStickyFaults(stickyFaults);

    DataLogManager.log(String.format("  APIError .......... %5s %5s", faults.APIError, ""));
    DataLogManager.log(String.format("  BootDuringEnable .. %5s %5s", faults.BootDuringEnable, ""));
    DataLogManager.log(String.format("  HardwareFault ..... %5s %5s", faults.HardwareFault, ""));
    DataLogManager.log(String.format("  ShortCircuit ...... %5s %5s", faults.ShortCircuit, ""));
    DataLogManager.log(String.format("  SoftwareFuse ...... %5s %5s", faults.SoftwareFuse, ""));
    DataLogManager.log(String.format("  ThermalFault ...... %5s %5s", faults.ThermalFault, ""));
    DataLogManager.log(String.format("  V5TooHigh ......... %5s %5s", faults.V5TooHigh, ""));
    DataLogManager.log(String.format("  V5TooLow .......... %5s %5s", faults.V5TooLow, ""));
    DataLogManager.log(String.format("  VBatTooHigh ....... %5s %5s", faults.VBatTooHigh, ""));
    DataLogManager.log(String.format("  VBatTooLow ........ %5s %5s", faults.VBatTooLow, ""));
  }

}
