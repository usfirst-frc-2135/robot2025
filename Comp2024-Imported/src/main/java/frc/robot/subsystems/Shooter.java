//
// Shooter Subystem - scores Notes into the Speaker
//
package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Shooter subsystem to control the shooter flywheel mechanisms and provide command factories
 */
public class Shooter extends SubsystemBase
{
  // Constants
  private static final String kSubsystemName     = "Shooter";

  private static final double kMOI               = 0.001;     // Simulation - Moment of Inertia
  private static final double kFlywheelScoreRPM  = 3300.0;    // RPM to score
  private static final double kFlywheelPassRPM   = 3000.0;    // RPM to pass
  private static final double kToleranceRPM      = 150.0;     // Tolerance band around target RPM

  private static final double kFlywheelGearRatio = (18.0 / 18.0);

  /** Shooter (speed) modes */
  private enum ShooterMode
  {
    REVERSE,    // Shooter runs in reverse direction to handle jams
    STOP,       // Shooter is stopped
    SCORE,      // Shooter ramped to an initial speed before shooting
    PASS        // Shooter slowed to passing speed
  }

  // Devices  objects
  private final TalonFX              m_lowerMotor            = new TalonFX(Ports.kCANID_ShooterLower);
  private final TalonFX              m_upperMotor            = new TalonFX(Ports.kCANID_ShooterUpper);

  // Simulation objects
  private final TalonFXSimState      m_lowerMotorSim         = new TalonFXSimState(m_lowerMotor);
  private final TalonFXSimState      m_upperMotorSim         = new TalonFXSimState(m_upperMotor);
  private final FlywheelSim          m_lowerFlywheelSim      = new FlywheelSim(DCMotor.getFalcon500(1), kFlywheelGearRatio, kMOI);
  private final FlywheelSim          m_upperFlywheelSim      = new FlywheelSim(DCMotor.getFalcon500(1), kFlywheelGearRatio, kMOI);

  // Status signals
  private final StatusSignal<Double> m_lowerVelocity;   // Default 4Hz (250ms)
  private final StatusSignal<Double> m_upperVelocity;   // Default 4Hz (250ms)

  // Declare module variables
  private boolean                    m_shooterValid;
  private boolean                    m_isAttargetRPM         = false; // Indicates flywheel RPM is close to target
  private boolean                    m_isAttargetRPMPrevious = false;

  private double                     m_targetRPM;            // Requested target flywheel RPM
  private double                     m_lowerRPM;             // Current lower RPM
  private double                     m_upperRPM;             // Current upper RPM

  private VelocityVoltage            m_requestVelocity       = new VelocityVoltage(0.0);
  private VoltageOut                 m_requestVolts          = new VoltageOut(0.0);
  private LinearFilter               m_lowerFlywheelFilter   = LinearFilter.singlePoleIIR(0.060, 0.020);
  private LinearFilter               m_upperFlywheelFilter   = LinearFilter.singlePoleIIR(0.060, 0.020);

  // Shuffleboard objects
  ShuffleboardTab                    m_shooterTab            = Shuffleboard.getTab(kSubsystemName);
  ShuffleboardLayout                 m_lowerList             =
      m_shooterTab.getLayout("Lower", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
  GenericEntry                       m_lowerValidEntry       = m_lowerList.add("lowerValid", false).getEntry( );
  GenericEntry                       m_lowerSpeedEntry       = m_lowerList.add("lowerSpeed", 0.0).getEntry( );

  ShuffleboardLayout                 m_upperList             =
      m_shooterTab.getLayout("Upper", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 3);
  GenericEntry                       m_upperValidEntry       = m_upperList.add("upperValid", false).getEntry( );
  GenericEntry                       m_upperSpeedEntry       = m_upperList.add("upperSpeed", 0.0).getEntry( );

  ShuffleboardLayout                 m_statusList            =
      m_shooterTab.getLayout("Status", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 3);
  GenericEntry                       m_atDesiredRPMEntry     = m_statusList.add("atDesiredRPM", false).getEntry( );
  GenericEntry                       m_targetRPMEntry        = m_statusList.add("targetRPM", 0.0).getEntry( );
  GenericEntry                       m_flywheelScoreEntry    = m_statusList.add("flywheelRPM", 0.0).getEntry( );

  /****************************************************************************
   * 
   * Constructor
   */
  public Shooter( )
  {
    setName("Shooter");
    setSubsystem("Shooter");

    boolean lowerValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_lowerMotor, kSubsystemName + "Lower", CTREConfigs6.shooterFXConfig( ));
    boolean upperValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_upperMotor, kSubsystemName + "Upper", CTREConfigs6.shooterFXConfig( ));
    m_lowerValidEntry.setBoolean(lowerValid);
    m_upperValidEntry.setBoolean(upperValid);
    m_shooterValid = lowerValid && upperValid;

    m_lowerVelocity = m_lowerMotor.getRotorVelocity( );
    m_upperVelocity = m_upperMotor.getRotorVelocity( );

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_lowerVelocity, m_upperVelocity);

    StatusSignal<Double> m_lowerSupplyCur = m_lowerMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Double> m_lowerStatorCur = m_lowerMotor.getStatorCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Double> m_upperSupplyCur = m_upperMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Double> m_upperStatorCur = m_upperMotor.getStatorCurrent( ); // Default 4Hz (250ms)

    DataLogManager.log(String.format(
        "%s: Update (Hz) lowerVelocity: %.1f upperVelocity: %.1f lowerSupplyCur: %.1f lowerStatorCur: %.1f upperSupplyCur: %.1f upperStatorCur: %.1f",
        getSubsystem( ), m_lowerVelocity.getAppliedUpdateFrequency( ), m_upperVelocity.getAppliedUpdateFrequency( ),
        m_lowerSupplyCur.getAppliedUpdateFrequency( ), m_lowerStatorCur.getAppliedUpdateFrequency( ),
        m_upperSupplyCur.getAppliedUpdateFrequency( ), m_upperStatorCur.getAppliedUpdateFrequency( )));

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

    // Calculate flywheel RPM and display on dashboard
    if (m_shooterValid)
    {
      BaseStatusSignal.refreshAll(m_lowerVelocity, m_upperVelocity);
      m_lowerRPM = m_lowerFlywheelFilter.calculate((m_lowerVelocity.getValue( ) * 60.0));
      m_upperRPM = m_upperFlywheelFilter.calculate((m_upperVelocity.getValue( ) * 60.0));
      m_lowerSpeedEntry.setDouble(m_lowerRPM);
      m_upperSpeedEntry.setDouble(m_upperRPM);

      m_isAttargetRPM = ((m_lowerRPM > kToleranceRPM) && MathUtil.isNear(m_targetRPM, m_lowerRPM, kToleranceRPM))
          && ((m_upperRPM > kToleranceRPM) && MathUtil.isNear(m_targetRPM, m_upperRPM, kToleranceRPM));
      m_atDesiredRPMEntry.setBoolean(m_isAttargetRPM);

      if (m_isAttargetRPM != m_isAttargetRPMPrevious)
      {
        DataLogManager.log(String.format("%s: At desired speed now: %.1f", getSubsystem( ), m_targetRPM));
        m_isAttargetRPMPrevious = m_isAttargetRPM;
      }
    }

    m_targetRPMEntry.setDouble(m_targetRPM);
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input flywheel voltage from the motor setting
    m_lowerMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_upperMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_lowerFlywheelSim.setInput(m_lowerMotorSim.getMotorVoltage( ));
    m_upperFlywheelSim.setInput(m_upperMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_lowerFlywheelSim.update(0.020);
    m_upperFlywheelSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_lowerMotorSim.setRotorVelocity(m_lowerFlywheelSim.getAngularVelocityRPM( ) / 60.0);
    m_upperMotorSim.setRotorVelocity(m_upperFlywheelSim.getAngularVelocityRPM( ) / 60.0);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_lowerFlywheelSim.getCurrentDrawAmps( )));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_upperFlywheelSim.getCurrentDrawAmps( )));
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    m_flywheelScoreEntry.setDouble(kFlywheelScoreRPM);

    ShuffleboardLayout cmdList = m_shooterTab.getLayout("Commands", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 3)
        .withProperties(Map.of("Label position", "HIDDEN"));
    cmdList.add("ShRunScore", getShooterScoreCommand( ));
    cmdList.add("ShRunStop", getShooterStopCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));
    setShooterMode(ShooterMode.STOP);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_lowerMotor, "ShooterLower");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_upperMotor, "ShooterUpper");
    m_lowerMotor.clearStickyFaults( );
    m_upperMotor.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set shooter speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  private void setShooterMode(ShooterMode mode)
  {
    DataLogManager.log(String.format("%s: Set shooter mode to %s", getSubsystem( ), mode));

    // Select the shooter RPM for the requested mode - NEVER NEGATIVE when running!
    switch (mode)
    {
      default :
        DataLogManager.log(String.format("%s: Shooter mode is invalid: %s", getSubsystem( ), mode));
      case STOP :
        m_targetRPM = 0.0;
        break;
      case SCORE :
        m_targetRPM = m_flywheelScoreEntry.getDouble(0.0);
        break;
      case PASS :
        m_targetRPM = kFlywheelPassRPM;
        break;
    }

    double rotPerSecond = m_targetRPM / 60.0;
    if (m_shooterValid)
    {
      if (m_targetRPM > 100.0)
        setShooterVelocity(rotPerSecond);
      else
        setShooterStopped( );
    }
    DataLogManager.log(String.format("%s: Target rpm is %.1f rps %.1f", getSubsystem( ), m_targetRPM, rotPerSecond));
  }

  /****************************************************************************
   * 
   * Set shooter motors to requested velocity
   * 
   * @param rps
   *          rotations per second
   */
  private void setShooterVelocity(double rps)
  {
    m_lowerMotor.setControl(m_requestVelocity.withVelocity(Conversions.rotationsToInputRotations(rps, kFlywheelGearRatio)));
    m_upperMotor.setControl(m_requestVelocity.withVelocity(Conversions.rotationsToInputRotations(rps, kFlywheelGearRatio)));
  }

  /****************************************************************************
   * 
   * Set shooter motors to stopped
   */
  private void setShooterStopped( )
  {
    m_lowerMotor.setControl(m_requestVolts);
    m_upperMotor.setControl(m_requestVolts);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return shooter speed check against target RPM
   * 
   * @return true if shooter is at target RPM
   */
  public boolean isAtTargetRPM( )
  {
    return m_isAttargetRPM;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create shooter command based on passed mode
   * 
   * @param mode
   *          shooter mode that detemines speed
   * @return instant command that changes shooter motors
   */
  private Command getShooterCommand(ShooterMode mode)
  {
    return new InstantCommand(        // Command that runs exactly once
        ( ) -> setShooterMode(mode),  // Method to call
        this                          // Subsystem requirement
    );
  }

  /****************************************************************************
   * 
   * Create shooter mode command for passing
   * 
   * @return instant command that runs shooter motors for scoring
   */
  public Command getShooterPassCommand( )
  {
    return getShooterCommand(ShooterMode.PASS).withName("ShooterPassNote");
  }

  /****************************************************************************
   * 
   * Create shooter mode command for feeding notes
   * 
   * @return instant command that runs shooter motors for feeding
   */
  public Command getShooterFeedCommand( )
  {
    return getShooterCommand(ShooterMode.PASS).withName("ShooterScore");
  }

  /****************************************************************************
   * 
   * Create shooter mode command for scoring
   * 
   * @return instant command that runs shooter motors for scoring
   */
  public Command getShooterScoreCommand( )
  {
    return getShooterCommand(ShooterMode.SCORE).withName("ShooterScore");
  }

  /****************************************************************************
   * 
   * Create shooter mode command to stop motors
   * 
   * @return instant command that stops shooter motors
   */
  public Command getShooterStopCommand( )
  {
    return getShooterCommand(ShooterMode.STOP).withName("ShooterStop");
  }

}
