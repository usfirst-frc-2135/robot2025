// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevSim
{
  private static final double   kGearRatio          = 40.0;
  private static final double   kCarriageMassKg     = 2.0;
  private static final double   kDrumDiameterMeters = 2.0 / 39.37;  // Drum diameter in meters (make meter = rotation)
  private static final double   kLengthMeters       = 10.0;         // Maximum length in meters
  private static final double   kDrumCircumMeters   = kDrumDiameterMeters * Math.PI;      // Drum diameter in meters
  // private static final double   kRolloutRatioMeters = kDrumCircumMeters / kGearRatio;     // Meters per shaft rotation

  private final ElevatorSim     m_elevatorSim       = new ElevatorSim(DCMotor.getVex775Pro(1), kGearRatio, kCarriageMassKg,
      kDrumDiameterMeters / 2, -kLengthMeters, kLengthMeters, false, 0.0);

  private TalonSRXSimCollection m_motorSim;
  private double                m_cpr;

  /**
   * Creates a new Elevator Simulation class.
   *
   * @param motorSim
   *          The motor simulation.
   * @param encoderCPR
   *          Encoder counts per revolution.
   */
  public ElevSim(TalonSRXSimCollection motorSim, double encoderCPR)
  {
    // Connect simulation object to the Talon SRX
    m_motorSim = motorSim;
    m_cpr = encoderCPR;
  }

  public void periodic( )
  {
    // Set input motor voltage from the motor setting
    m_motorSim.setBusVoltage(RobotController.getInputVoltage( ));
    m_elevatorSim.setInput(m_motorSim.getMotorOutputLeadVoltage( ));

    // update for 20 msec loop
    m_elevatorSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.setQuadratureRawPosition((int) (m_cpr * m_elevatorSim.getPositionMeters( ) / kDrumCircumMeters));
    m_motorSim.setQuadratureVelocity((int) (m_cpr * (m_elevatorSim.getVelocityMetersPerSecond( ) / kDrumCircumMeters) / 10));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps( )));

    SmartDashboard.putNumber("SIM-motorVolts", m_motorSim.getMotorOutputLeadVoltage( ));    // Voltage applied to motor
    SmartDashboard.putNumber("SIM-elevPos", m_elevatorSim.getPositionMeters( ));            // Get elevator position from simulation (meters)
    SmartDashboard.putNumber("SIM-elevVel", m_elevatorSim.getVelocityMetersPerSecond( ));   // Get elevator velocity from simulation (mps)
  }

  public void reset( )
  {
    m_motorSim.setQuadratureRawPosition((int) (m_cpr * m_elevatorSim.getPositionMeters( ) / kDrumCircumMeters));
    m_motorSim.setQuadratureVelocity((int) (m_cpr * (m_elevatorSim.getVelocityMetersPerSecond( ) / kDrumCircumMeters) / 10));
    m_elevatorSim.setState(0.0, 0.0);
    m_elevatorSim.setInput(0.0);
  }

}
