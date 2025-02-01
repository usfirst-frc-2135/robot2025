// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ExampleSmartMotorController implements MotorController
{
  public enum PIDMode
  {
    kPosition, kVelocity, kMovementWitchcraft
  }

  // private final static double ks          = 0.0;   // Volts to overcome static friction
  // private final static double kf          = 0.0;   // Volts per velocity unit
  private final static double   m_kp = 0.8;           // (native units) 10% * 102.3 / 1023
  private final static double   m_ki = 0.0;
  private final static double   m_kd = 0.0;

  private int                   m_port;
  private WPI_TalonSRX          m_srxMotor;
  private TalonSRXSimCollection m_srxMotorSim;
  private double                m_encoderCPR;
  private double                m_rotations;
  private double                m_rotPerSec;

  /**
   * Creates a new ExampleSmartMotorController.
   *
   * @param port
   *          The port for the controller.
   * @param encoderCPR
   *          The counts per rotation for the attached encoder.
   */
  //  @SuppressWarnings("PMD.UnusedFormalParameter")
  public ExampleSmartMotorController(int port, double encoderCPR)
  {
    m_port = port;
    m_encoderCPR = encoderCPR;

    // Create the Talon SRX object and the attached CTRE Mag encoder
    m_srxMotor = new WPI_TalonSRX(port);
    m_srxMotorSim = m_srxMotor.getSimCollection( );

    m_srxMotor.configFactoryDefault( );
    m_srxMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_srxMotor.selectProfileSlot(0, 0);

    m_srxMotor.config_kP(0, m_kp);
    m_srxMotor.config_kI(0, m_ki);
    m_srxMotor.config_kD(0, m_kd);
  }

  /**
   * Periodic processing for this motor/controller
   * 
   */
  public void periodic( )
  {
    // This method will be called once per scheduler run

    m_rotations = countsToRotations(m_srxMotor.getSelectedSensorPosition(0));
    m_rotPerSec = countsToRotations(m_srxMotor.getSelectedSensorVelocity(0) * 10);

    SmartDashboard.putNumber("SRX" + m_port + "-Rotations", m_rotations);                                 // Output shaft distance (rotations)
    SmartDashboard.putNumber("SRX" + m_port + "-Velocity", m_rotPerSec);                                  // Output shaft velocity (rps)
    SmartDashboard.putNumber("SRX" + m_port + "-normError", m_srxMotor.getClosedLoopError( ) / m_encoderCPR); // Normalized error in shaft rotations 
    if (m_srxMotor.getControlMode( ) == ControlMode.Position || m_srxMotor.getControlMode( ) == ControlMode.Velocity)
      SmartDashboard.putNumber("SRX" + m_port + "-target", m_srxMotor.getClosedLoopTarget( ));              // Normalized error in shaft rotations 
  }

  /**
   * Return the motor simulation object for this controller
   *
   * @return motor simulation object
   */
  public TalonSRXSimCollection getMotorSimulation( )
  {
    return m_srxMotorSim;
  }

  /**
   * Converts shaft rotations to encoder counts
   *
   * @param rotation
   *          The rotation value to be converted.
   * @return encoderCounts
   */
  private double rotationsToCounts(double rotation)
  {
    return rotation * m_encoderCPR;
  }

  /**
   * Converts encoder counts to shaft rotations
   *
   * @param encoderCounts
   *          The count value to be converted.
   * @return rotations
   */
  private double countsToRotations(double encoderCounts)
  {
    return encoderCounts / m_encoderCPR;
  }

  /**
   * Set the setpoint of the smart controller in PID mode.
   *
   * @param mode
   *          The mode of the PID controller.
   * @param setpoint
   *          The controller setpoint.
   * @param arbFeedforward
   *          An arbitrary feedforward output (from -1 to 1).
   */
  public void setSetpoint(PIDMode mode, double setpoint, double arbFeedforward)
  {
    ControlMode controlMode;

    switch (mode)
    {
      default :
      case kPosition :  // Position PID
        controlMode = ControlMode.Position;
        break;

      case kVelocity :  // Velocity PID
        controlMode = ControlMode.Velocity;
        setpoint /= 10; // Adjust for CTRE units
        break;

      case kMovementWitchcraft :  // Motion Magic profile TODO: not yet implemented
        controlMode = ControlMode.MotionMagic;
        break;
    }

    // Talon SRX is before gearbox, but CTRE mag encoder is after it (multiply by gear ratio)
    m_srxMotor.set(controlMode, rotationsToCounts(setpoint));
  }

  /**
   * Places this motor controller in follower mode.
   *
   * @param leader
   *          The leader to follow.
   */
  public void follow(ExampleSmartMotorController leader)
  {}

  /**
   * Returns the encoder position in rotations.
   *
   * @return The current encoder position in rotations.
   */
  public double getEncoderPosition( )
  {
    return m_rotations;
  }

  /**
   * Returns the encoder velocity.
   *
   * @return The current encoder velocity in rotations per second.
   */
  public double getEncoderVelocity( )
  {
    return countsToRotations(m_srxMotor.getSelectedSensorVelocity(0) * 10);
  }

  /**
   * Resets the encoder to zero distance.
   * 
   */
  public void resetEncoder( )
  {
    m_srxMotor.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Set motor to a known percent output value.
   *
   * @param percentOutput
   *          Percent output in range [-1.0, 1.0].
   */
  @Override
  public void set(double percentOutput)
  {
    m_srxMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Returns the motor setting in use.
   *
   * @return percentOutput in range [-1.0, 1.0].
   */
  @Override
  public double get( )
  {
    return m_srxMotor.getMotorOutputPercent( );
  }

  /**
   * Set motor inverted state.
   *
   * @param isInverted
   *          Invert motor if true
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    m_srxMotor.setInverted(isInverted);
  }

  /**
   * Get motor inverted state.
   *
   * @return invertedState
   */
  @Override
  public boolean getInverted( )
  {
    return m_srxMotor.getInverted( );
  }

  /**
   * Set motor to the disabled state.
   * 
   */
  @Override
  public void disable( )
  {
    m_srxMotor.set(ControlMode.Disabled, 0);
  }

  /**
   * Set motor to the stopped state.
   * 
   */
  @Override
  public void stopMotor( )
  {
    set(0.0);
  }

}
