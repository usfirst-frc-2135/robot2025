
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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
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
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 45.0;           // Amps
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30.0;      // Amps
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.080;      // Seconds
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits.StatorCurrentLimit = 120.0;          // Amps
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // elevatorConfig.Feedback.*

    // Hardware limit switches - NONE
    // elevatorConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 82.17;     // Rotations / second
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 300.0;       // Rotations / second ^ 2
    elevatorConfig.MotionMagic.MotionMagicJerk = 3200;                // Rotations / second ^ 3

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
    elevatorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    elevatorConfig.Slot0.kS = 0.075;                                  // Feedforward: Voltage or duty cycle to overcome static friction
    elevatorConfig.Slot0.kG = 0.325;                                  // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    elevatorConfig.Slot0.kV = 0.1241;                                 // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    elevatorConfig.Slot0.kP = 0.75;                                   // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    elevatorConfig.Slot0.kI = 0.0;                                    // Feedback: Voltage or duty cycle per accumulated unit
    elevatorConfig.Slot0.kD = 0.09;                                   // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

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

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    wristRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 82.17 / gearRatio;  // 1.67 Rotations / second
    wristRotaryConfig.MotionMagic.MotionMagicAcceleration = 950.0 / gearRatio;    // 19.3 Rotations / second ^ 2
    wristRotaryConfig.MotionMagic.MotionMagicJerk = 0 / gearRatio;                // 500 Rotations / second ^ 3

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
    wristRotaryConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    wristRotaryConfig.Slot0.kS = 0.05;                                // Feedforward: Voltage or duty cycle to overcome static friction (Measured 0.7V) 
    wristRotaryConfig.Slot0.kG = -0.25;                               // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward) (Measured -0.18 to -0.32)
    wristRotaryConfig.Slot0.kV = 0.1241;                              // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    // NOTE: Motion Magic settings are scaled by gear ration when using a FusecCANCoder
    wristRotaryConfig.Slot0.kP = 1.62 * gearRatio;                    // 79.75 Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    wristRotaryConfig.Slot0.kI = 0.0 * gearRatio;                     // 0.0   Feedback: Voltage or duty cycle per accumulated unit
    wristRotaryConfig.Slot0.kD = 0.0609 * gearRatio;                  // 2.998 Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

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

    ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.25;
    if (Robot.isReal( ))
      ccConfig.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? -0.5345 : -0.212727;
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
    clawRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;         // Amps
    clawRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;    // Amps
    clawRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;    // Seconds
    clawRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    clawRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;        // Amps
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
    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    //                                                                Wrist Upward was x.x V, Elevator Downward was x.x.
    //                                                                  kG = (0.40 + 0.25) / 2
    //                                                                  kS = (0.40 - 0.25) / 2
    // clawRollerConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    // clawRollerConfig.Slot0.kS = 0.0;                                  // Feedforward: Voltage or duty cycle to overcome static friction
    // clawRollerConfig.Slot0.kG = 0.0;                                  // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    // clawRollerConfig.Slot0.kV = 0.1241;                               // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    // clawRollerConfig.Slot0.kP = 0.9;                                 // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    // clawRollerConfig.Slot0.kI = 0.0;                                 // Feedback: Voltage or duty cycle per accumulated unit
    // clawRollerConfig.Slot0.kD = 0.0;                                 // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

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

    crConfig.ProximityParams.ProximityThreshold = 0.1;                // Proximity distance in meters (about 8 inches)
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
