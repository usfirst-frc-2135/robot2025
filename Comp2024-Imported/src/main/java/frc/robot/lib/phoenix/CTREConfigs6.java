
// Phoenix 6 configurations

package frc.robot.lib.phoenix;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
   * Intake rotary motor - Falcon 500
   */
  public static TalonFXConfiguration intakeRotaryFXConfig(double min, double max, int ccPort, double gearRatio)
  {
    TalonFXConfiguration inRotaryConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    inRotaryConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    inRotaryConfig.CurrentLimits.SupplyCurrentThreshold = 25.0;   // Amps
    inRotaryConfig.CurrentLimits.SupplyTimeThreshold = 0.001;     // Seconds
    inRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRotaryConfig.CurrentLimits.StatorCurrentLimit = 100.0;       // Amps
    inRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    inRotaryConfig.Feedback.FeedbackRemoteSensorID = ccPort;
    inRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    inRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    inRotaryConfig.Feedback.RotorToSensorRatio = gearRatio;

    // Hardware limit switches - NONE
    // inRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    inRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0 / gearRatio;  // Rotations / second
    inRotaryConfig.MotionMagic.MotionMagicAcceleration = 220.0 / gearRatio;    // Rotations / second ^ 2
    inRotaryConfig.MotionMagic.MotionMagicJerk = 1600.0 / gearRatio;           // Rotations / second ^ 3

    // Motor output settings
    inRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;  // Percentage
    inRotaryConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    inRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // inRotaryConfig.OpenLoopRamps.*                             // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    inRotaryConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    inRotaryConfig.Slot0.kS = 0.0;                                // Feedforward: Voltage or duty cylce to overcome static friction
    inRotaryConfig.Slot0.kG = -0.50;                               // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward)
    inRotaryConfig.Slot0.kV = 0.1129;                             // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    inRotaryConfig.Slot0.kP = 3.6 * gearRatio;                    // Feedback: Voltage or duty cycle per velocity error (velocity modes)
    inRotaryConfig.Slot0.kI = 0.0 * gearRatio;                    // Feedback: Voltage or duty cycle per accumulated error
    inRotaryConfig.Slot0.kD = 0.0 * gearRatio;                    // Feedback: Voltage or duty cycle per unit of acceleration error (velocity modes)

    // Software limit switches
    inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;  // Rotations
    // inRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;  // Rotations
    // inRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return inRotaryConfig;
  }

  /****************************************************************************
   * 
   * Intake rotary CANcoder
   */
  public static CANcoderConfiguration intakeRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    double kQuarterRotation = 0.25;
    double CompRobotOffset = -0.015;

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    if (Robot.isReal( ))
      config.MagnetSensor.MagnetOffset =
          (Robot.isComp( )) ? (-0.311768 - kQuarterRotation + CompRobotOffset) : (0.1184 - kQuarterRotation);
    else
      config.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default in rotations

    return config;
  }

  /****************************************************************************
   * 
   * Shooter motors - Falcon 500 (2)
   */
  public static TalonFXConfiguration shooterFXConfig( )
  {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    shooterConfig.CurrentLimits.SupplyCurrentLimit = 35.0;        // Amps
    shooterConfig.CurrentLimits.SupplyCurrentThreshold = 35.0;    // Amps
    shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.001;      // Seconds
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // shooterConfig.CurrentLimits.StatorCurrentLimit = 100.0;        // Amps
    // shooterConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // shooterConfig.Feedback.*
    // shooterConfig.HardwareLimitSwitch.*
    // shooterConfig.MotionMagic.*

    // Motor output settings
    // shooterConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // shooterConfig.OpenLoopRamps.*                                // Seconds to ramp

    // Slot settings
    // shooterConfig.Slot0.GravityType = *;                         // Feedforward: Mechanism is an elevator or arm
    shooterConfig.Slot0.kS = 0.0;                                   // Feedforward: Voltage or duty cylce to overcome static friction
    shooterConfig.Slot0.kG = 0.0;                                   // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward)
    shooterConfig.Slot0.kV = 0.1140;                                // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    shooterConfig.Slot0.kP = 0.25;                                  // Voltage or duty cycle per velocity error (velocity modes)
    shooterConfig.Slot0.kI = 0.0;                                   // Voltage or duty cycle per accumulated error
    shooterConfig.Slot0.kD = 0.0;                                   // Voltage or duty cycle per unit of acceleration error (velocity modes)

    // shooterConfig.SoftwareLimitSwitch.*

    return shooterConfig;
  }

  /****************************************************************************
   * 
   * Feeder rotary motor - Falcon 500
   */
  public static TalonFXConfiguration feederRotaryFXConfig(double min, double max, int ccPort, double gearRatio)
  {
    TalonFXConfiguration fdRotaryConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // fdRotaryConfig.ClosedLoopGeneral.*
    // fdRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    fdRotaryConfig.CurrentLimits.SupplyCurrentLimit = 32.0;       // Amps
    fdRotaryConfig.CurrentLimits.SupplyCurrentThreshold = 32.0;   // Amps
    fdRotaryConfig.CurrentLimits.SupplyTimeThreshold = 0.001;     // Seconds
    fdRotaryConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    fdRotaryConfig.CurrentLimits.StatorCurrentLimit = 150.0;       // Amps
    fdRotaryConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    fdRotaryConfig.Feedback.FeedbackRemoteSensorID = ccPort;
    fdRotaryConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fdRotaryConfig.Feedback.SensorToMechanismRatio = 1.0;
    fdRotaryConfig.Feedback.RotorToSensorRatio = gearRatio;

    // Hardware limit switches - NONE
    // fdRotaryConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    fdRotaryConfig.MotionMagic.MotionMagicCruiseVelocity = 30.0 / gearRatio;  // Rotations / second
    fdRotaryConfig.MotionMagic.MotionMagicAcceleration = 90.0 / gearRatio;    // Rotations / second ^ 2
    fdRotaryConfig.MotionMagic.MotionMagicJerk = 360.0 / gearRatio;           // Rotations / second ^ 3

    // Motor output settings
    fdRotaryConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;  // Percentage
    fdRotaryConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    fdRotaryConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // fdRotaryConfig.OpenLoopRamps.*                             // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    fdRotaryConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    fdRotaryConfig.Slot0.kS = 0.0;                                // Feedforward: Voltage or duty cylce to overcome static friction
    fdRotaryConfig.Slot0.kG = -0.50;                                // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward) 
    fdRotaryConfig.Slot0.kV = 0.1129;                             // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    fdRotaryConfig.Slot0.kP = 2.4 * gearRatio;                    // Feedback: Voltage or duty cycle per velocity error (velocity modes)
    fdRotaryConfig.Slot0.kI = 0.0 * gearRatio;                    // Feedback: Voltage or duty cycle per accumulated error
    fdRotaryConfig.Slot0.kD = 0.0 * gearRatio;                    // Feedback: Voltage or duty cycle per unit of acceleration error (velocity modes)

    // Software limit switches
    fdRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;  // Rotations
    // fdRotaryConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    fdRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;  // Rotations
    // fdRotaryConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return fdRotaryConfig;
  }

  /****************************************************************************
   * 
   * Feeder rotary CANcoder
   */
  public static CANcoderConfiguration feederRotaryCancoderConfig( )
  {
    CANcoderConfiguration config = new CANcoderConfiguration( );
    double kQuarterRotation = 0.25;

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    if (Robot.isReal( ))
      config.MagnetSensor.MagnetOffset = (Robot.isComp( )) ? (0.059814 - kQuarterRotation) : (-0.2581 - kQuarterRotation);
    else
      config.MagnetSensor.MagnetOffset = -0.25; // Simulated CANcoder default in rotations

    return config;
  }

  /****************************************************************************
   * 
   * Climber motors (2) - Falcon 500
   */
  public static TalonFXConfiguration climberFXConfig(double min, double max)
  {
    TalonFXConfiguration climberConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // exConfig.ClosedLoopGeneral.*
    // exConfig.ClosedLoopRamps.*

    // Current limit settings
    climberConfig.CurrentLimits.SupplyCurrentLimit = 80.0;        // Amps
    climberConfig.CurrentLimits.SupplyCurrentThreshold = 80.0;    // Amps
    climberConfig.CurrentLimits.SupplyTimeThreshold = 0.001;      // Seconds
    climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberConfig.CurrentLimits.StatorCurrentLimit = 800.0;        // Amps
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // climberConfig.Feedback.*

    // Hardware limit switches - NONE
    // climberConfig.HardwareLimitSwitch.*

    // Motion Magic settings
    climberConfig.MotionMagic.MotionMagicCruiseVelocity = 79.75;  // Rotations / second
    climberConfig.MotionMagic.MotionMagicAcceleration = 159.5;    // Rotations / second ^ 2
    climberConfig.MotionMagic.MotionMagicJerk = 3544;             // Rotations / second ^ 3

    // Motor output settings
    climberConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;   // Percentage
    climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // climberConfig.OpenLoopRamps.*                              // Seconds to ramp

    // Slot settings
    climberConfig.Slot0.kS = 0.0;                                 // Feedforward: Voltage or duty cylce to overcome static friction
    climberConfig.Slot0.kG = 0.0;                                 // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward)
    climberConfig.Slot0.kV = 0.1129;                              // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    climberConfig.Slot0.kP = 9.60;                                // Feedback: Voltage or duty cycle per velocity error (velocity modes)
    climberConfig.Slot0.kI = 0.0;                                 // Feedback: Voltage or duty cycle per accumulated error
    climberConfig.Slot0.kD = 0.0;                                 // Feedback: Voltage or duty cycle per unit of acceleration error (velocity modes)

    // Software limit switches
    // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;   // Rotations
    // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;   // Rotations
    // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return climberConfig;
  }
}
