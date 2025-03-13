
// Phoenix 6 configurations

package frc.robot.lib.phoenix;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import frc.robot.Robot;

/****************************************************************************
 * 
 * CTRE configuration structure for v6 devices
 */
public final class CTREConfigs6
{

  // Swerve module configs are built into swerve subsystem

  /****************************************************************************
   * 
   * Elevator motors (2 - one for left and right) - Kraken X60
   * 
   * @param inverted
   *          motor inversion request
   * @param min
   *          minimum deployment distance
   * @param max
   *          maximum deployement distance (must be greater than min)
   */
  public static TalonFXConfiguration elevatorFXConfig(boolean inverted, double min, double max)
  {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // exConfig.ClosedLoopGeneral.*
    // exConfig.ClosedLoopRamps.*

    // Current limit settings
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 25.0;           // Amps
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 28.0;      // Amps
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;      // Seconds
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits.StatorCurrentLimit = 400.0;          // Amps
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // elevatorConfig.Feedback.*

    // Hardware limit switches - NONE
    // elevatorConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 72.50 * 0.75;  // Rotations / second // TODO: speed up?
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 241.7 * 0.75;    // Rotations / second ^ 2
    elevatorConfig.MotionMagic.MotionMagicJerk = 2417 * 0.75;             // Rotations / second ^ 3

    // Motor output settings
    elevatorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;      // Percentage
    elevatorConfig.MotorOutput.Inverted = (inverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // elevatorConfig.OpenLoopRamps.*                                 // Seconds to ramp

    // Slot settings
    //                                                                Elevator Upward was 0.40 V, Elevator Downward was 0.25.
    //                                                                  kG = (0.40 + 0.25) / 2
    //                                                                  kS = (0.40 - 0.25) / 2
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.Slot0.kS = 0.075;                                  // Feedforward: Voltage or duty cycle to overcome static friction
    elevatorConfig.Slot0.kG = 0.325;                                  // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    elevatorConfig.Slot0.kV = 0.1241;                                 // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    elevatorConfig.Slot0.kP = 9.60;                                   // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    elevatorConfig.Slot0.kI = 0.0;                                    // Feedback: Voltage or duty cycle per accumulated unit
    elevatorConfig.Slot0.kD = 0.0;                                    // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min; // Rotations
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max; // Rotations
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return elevatorConfig;
  }

  /****************************************************************************
   * 
   * Manipulator wrist rotary motor - Kraken X60
   * 
   * @param min
   *          minimum angle of rotation
   * @param max
   *          maximum angle of rotation (must be greater than min)
   * @param ccPort
   *          CANcoder port number
   * @param gearRatio
   *          gear box ratio
   */
  public static TalonFXConfiguration wristRotaryFXConfig(double min, double max, int ccPort, double gearRatio)
  {
    TalonFXConfiguration wristRotaryConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // wristRotaryConfig.ClosedLoopGeneral.*
    // wristRotaryConfig.ClosedLoopRamps.*                            // Seconds to ramp

    // Current limit settings
    wristRotaryConfig.CurrentLimits.SupplyCurrentLimit = 40.0;        // Amps
    wristRotaryConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;   // Amps
    wristRotaryConfig.CurrentLimits.SupplyCurrentLowerTime = 0.050;   // Seconds
    wristRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    wristRotaryConfig.CurrentLimits.StatorCurrentLimit = 400.0;       // Amps
    wristRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    wristRotaryConfig.Feedback.FeedbackRemoteSensorID = ccPort;
    wristRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    wristRotaryConfig.Feedback.RotorToSensorRatio = gearRatio;

    // Hardware limit switches - NONE
    // wristRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio // TODO: wrist is temporarily slowed until we get it tuned
    wristRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 62.83 / gearRatio * 0.75;  // Rotations / second  // TODO: speed up?
    wristRotaryConfig.MotionMagic.MotionMagicAcceleration = 241.7 / gearRatio * 0.75;   // Rotations / second ^ 2
    wristRotaryConfig.MotionMagic.MotionMagicJerk = 2417.0 / gearRatio * 0.75;          // Rotations / second ^ 3

    // Motor output settings
    wristRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;   // Percentage
    wristRotaryConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // wristRotaryConfig.OpenLoopRamps.*                              // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    //                                                                Wrist Upward was x.x V, Elevator Downward was x.x.
    //                                                                  kG = (0.40 + 0.25) / 2
    //                                                                  kS = (0.40 - 0.25) / 2
    wristRotaryConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    wristRotaryConfig.Slot0.kS = -0.065;                              // Feedforward: Voltage or duty cylce to overcome static friction
    wristRotaryConfig.Slot0.kG = -0.235;                              // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward)
    wristRotaryConfig.Slot0.kV = 0.1241;                              // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    wristRotaryConfig.Slot0.kP = 0.9 * gearRatio;                     // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    wristRotaryConfig.Slot0.kI = 0.0 * gearRatio;                     // Feedback: Voltage or duty cycle per accumulated unit
    wristRotaryConfig.Slot0.kD = 0.0 * gearRatio;                     // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    wristRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;  // Rotations
    wristRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;  // Rotations
    wristRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return wristRotaryConfig;
  }

  /****************************************************************************
   * 
   * Manipulator wrist rotary CANcoder
   */
  public static CANcoderConfiguration wristRotaryCANcoderConfig( )
  {
    CANcoderConfiguration ccConfig = new CANcoderConfiguration( );

    ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.25;
    if (Robot.isReal( ))
      ccConfig.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? -0.1078 : 0.0;
    else
      ccConfig.MagnetSensor.MagnetOffset = -0.25;                     // Simulated CANcoder default in rotations

    return ccConfig;
  }

  /****************************************************************************
   * 
   * Manipulator claw roller motor - Kraken X60
   * 
   * @param min
   *          minimum angle of rotation
   * @param max
   *          maximum angle of rotation (must be greater than min)
   * @param ccPort
   *          CANcoder port number
   * @param gearRatio
   *          gear box ratio
   */
  public static TalonFXConfiguration clawRollerFXConfig(int canRangeID)
  {
    TalonFXConfiguration clawRollerConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // clawRollerConfig.ClosedLoopGeneral.*
    // clawRollerConfig.ClosedLoopRamps.*                             // Seconds to ramp

    // Current limit settings
    clawRollerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;         // Amps
    clawRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;    // Amps
    clawRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;    // Seconds
    clawRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    clawRollerConfig.CurrentLimits.StatorCurrentLimit = 400.0;        // Amps
    clawRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // clawRollerConfig.Feedback.*

    // Hardware limit switches - CANrange
    clawRollerConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = canRangeID; // Stop coral on CANrange detection
    clawRollerConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    clawRollerConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    // clawRollerConfig.MotionMagic.*

    // Motor output settings
    clawRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    clawRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    clawRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // clawRollerConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // clawRollerConfig.Slot0.*                                       // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // clawRollerConfig.SoftwareLimitSwitch.*

    return clawRollerConfig;
  }

  /****************************************************************************
   * 
   * Manipulator coral CANRange detector
   */
  public static CANrangeConfiguration coralCANRangeConfig( )
  {
    CANrangeConfiguration crConfig = new CANrangeConfiguration( );

    crConfig.ProximityParams.ProximityThreshold = 0.2;                // Proximity distance in meters (about 8 inches)
    crConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    return crConfig;
  }

  /****************************************************************************
   * 
   * Manipulator algae CANRange detector
   */
  public static CANrangeConfiguration algaeCANRangeConfig( )
  {
    CANrangeConfiguration crConfig = new CANrangeConfiguration( );

    crConfig.ProximityParams.ProximityThreshold = 0.1;                // Proximity distance in meters (about 4 inches)
    crConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    return crConfig;
  }

}
