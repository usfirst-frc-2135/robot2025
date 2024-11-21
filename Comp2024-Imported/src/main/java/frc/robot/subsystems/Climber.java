//
// Climber Subystem - lifts the robot to hang onto the chain
//
package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Ports;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Climber subsystem class
 */
public class Climber extends SubsystemBase
{
  // Constants
  private static final String kSubsystemName       = "Climber";
  private static final double kGearRatio           = 16.0;    // Gear reduction
  private static final double kClimberLengthMeters = 0.5;     // Simulation
  private static final double kCarriageMassKg      = 2.0;     // Simulation
  private static final double kDrumDiameterInches  = 1.375;   // Drum diameter in inches
  private static final double kDrumRadiusMeters    = Units.inchesToMeters(kDrumDiameterInches) / 2;
  private static final double kRolloutRatio        = kDrumDiameterInches * Math.PI / kGearRatio; // inches per shaft rotation
  private static final double kCalibrateSpeedVolts = -1.0;    // Motor voltage during calibration
  private static final double kCalibrateStallAmps  = 25.0;    // Motor amps during calibration stall
  private static final double kManualSpeedVolts    = 3.0;     // Motor voltage during manual operation (joystick)

  private static final double kToleranceInches     = 0.5;     // Climber PID tolerance in inches
  private static final double kMMMoveTimeout       = 2.0;     // Seconds allowed for a Motion Magic movement
  private static final double kCalibrationTimeout  = 2.0;     // Max calibration time

  // Climber lengths - Motion Magic config parameters
  private static final double kLengthClimbed       = 0.0;     // By definition - Climber fully climbed
  private static final double kLengthFull          = 18.0;    // From Mech Design height needed to reach max chain
  private static final double kLengthChain         = 8.0;     // From Mech Design height needed to reach hanging chain

  private static final double kLengthMin           = 0.0;     // Climber minimum allowable length
  private static final double kLengthMax           = 21.0;    // Climber maximum allowable length (2" beyond high length)

  /** Climber manual move parameters */
  private enum ClimberMode
  {
    INIT,   // Initialize climber
    UP,     // Climber move upward
    STOP,   // Climber stop
    DOWN    // Climber move downward
  }

  // Device objects
  private final TalonFX              m_leftMotor            = new TalonFX(Ports.kCANID_ClimberL);
  private final TalonFX              m_rightMotor           = new TalonFX(Ports.kCANID_ClimberR);

  // Simulation objects
  private final TalonFXSimState      m_climberSim           = m_leftMotor.getSimState( );
  private final ElevatorSim          m_elevSim              = new ElevatorSim(DCMotor.getFalcon500(1), kGearRatio,
      kCarriageMassKg, kDrumRadiusMeters, -kLengthMax, kLengthMax, false, 0.0);

  // Mechanism2d
  private final Mechanism2d          m_climberMech          = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d  m_mechLigament         = m_climberMech.getRoot("Linear", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, kClimberLengthMeters, 0.0, 6, new Color8Bit(Color.kRed)));

  private final StatusSignal<Double> m_leftPosition;    // Default 4Hz (250ms)
  private final StatusSignal<Double> m_leftSupplyCur;   // Default 4Hz (250ms)
  private final StatusSignal<Double> m_leftStatorCur;   // Default 4Hz (250ms)
  private final StatusSignal<Double> m_rightPosition;   // Default 4Hz (250ms)
  private final StatusSignal<Double> m_rightSupplyCur;  // Default 4Hz (250ms)
  private final StatusSignal<Double> m_rightStatorCur;  // Default 4Hz (250ms)

  // Declare module variables 
  private boolean                    m_climberValid;                  // Health indicator for Falcon 
  private double                     m_leftCurInches        = 0.0;    // Current length in inches on left (default) side
  private double                     m_rightCurInches       = 0.0;    // Current length in inches on right side
  private double                     m_targetInches         = 0.0;    // Target length in inches

  // Calibration variables
  private Timer                      m_calibrateTimer       = new Timer( );
  private Debouncer                  m_leftStalled          = new Debouncer(0.100, DebounceType.kRising);
  private Debouncer                  m_rightStalled         = new Debouncer(0.100, DebounceType.kRising);
  private boolean                    m_leftCalibrated       = false;
  private boolean                    m_rightCalibrated      = false;

  // Manual mode config parameters
  private VoltageOut                 m_requestVolts         = new VoltageOut(0);
  private ClimberMode                m_mode                 = ClimberMode.INIT;  // Manual movement mode with joysticks
  private int                        m_hardStopCounter      = 0;

  // Motion Magic config parameters
  private MotionMagicVoltage         m_mmRequestVolts       = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                  m_mmWithinTolerance    = new Debouncer(0.060, DebounceType.kRising);
  private Timer                      m_mmMoveTimer          = new Timer( ); // Safety timer for movements
  private double                     m_totalArbFeedForward;  // Arbitrary feedforward added to counteract gravity
  private boolean                    m_mmMoveIsFinished;     // Movement has completed (within tolerance)

  // Shuffleboard objects
  private ShuffleboardTab            m_subsystemTab         = Shuffleboard.getTab(kSubsystemName);
  private ShuffleboardLayout         m_leftList             =
      m_subsystemTab.getLayout("Left", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3);
  private GenericEntry               m_leftValidEntry       = m_leftList.add("leftValid", false).getEntry( );
  private GenericEntry               m_leftInchesEntry      = m_leftList.add("leftInches", 0.0).getEntry( );
  // private GenericEntry               m_leftCLoopErrorEntry  = m_leftList.add("leftCLoopError", 0.0).getEntry( );

  private ShuffleboardLayout         m_rightList            =
      m_subsystemTab.getLayout("Right", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 3);
  private GenericEntry               m_rightValidEntry      = m_rightList.add("righttValid", false).getEntry( );
  private GenericEntry               m_rightInchesEntry     = m_rightList.add("rightInches", 0.0).getEntry( );
  // private GenericEntry               m_rightCLoopErrorEntry = m_rightList.add("rightCLoopError", 0.0).getEntry( );

  private ShuffleboardLayout         m_statusList           =
      m_subsystemTab.getLayout("Status", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 3);
  private GenericEntry               m_leftCalibratedEntry  = m_statusList.add("leftCalibrated", false).getEntry( );
  private GenericEntry               m_rightCalibratedEntry = m_statusList.add("rightCalibrated", false).getEntry( );
  private GenericEntry               m_targetInchesEntry    = m_statusList.add("targetInches", 0.0).getEntry( );

  /****************************************************************************
   * 
   * Constructor
   */
  public Climber( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Initialize climber motors
    boolean leftValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_leftMotor, kSubsystemName + "Left",
        CTREConfigs6.climberFXConfig(Units.degreesToRotations(kLengthMin), Units.degreesToRotations(kLengthMax)));
    boolean rightValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rightMotor, kSubsystemName + "Right",
        CTREConfigs6.climberFXConfig(Units.degreesToRotations(kLengthMin), Units.degreesToRotations(kLengthMax)));
    m_climberValid = leftValid && rightValid;
    m_leftValidEntry.setBoolean(leftValid);
    m_rightValidEntry.setBoolean(rightValid);

    m_leftPosition = m_leftMotor.getRotorPosition( );
    m_leftSupplyCur = m_leftMotor.getSupplyCurrent( );
    m_leftStatorCur = m_leftMotor.getStatorCurrent( );
    m_rightPosition = m_rightMotor.getRotorPosition( );
    m_rightSupplyCur = m_rightMotor.getSupplyCurrent( );
    m_rightStatorCur = m_rightMotor.getStatorCurrent( );

    if (m_climberValid)
    {
      m_rightMotor.setInverted(false);

      setClimberPosition(m_leftCurInches);

      // Status signals
      BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftPosition, m_rightPosition);

      DataLogManager.log(String.format(
          "%s: Update (Hz) leftPosition: %.1f rightPosition: %.1f leftSupplyCur: %.1f leftStatorCur: %.1f rightSupplyCur: %.1f rightStatorCur: %.1f",
          getSubsystem( ), m_leftPosition.getAppliedUpdateFrequency( ), m_rightPosition.getAppliedUpdateFrequency( ),
          m_leftSupplyCur.getAppliedUpdateFrequency( ), m_leftStatorCur.getAppliedUpdateFrequency( ),
          m_rightSupplyCur.getAppliedUpdateFrequency( ), m_rightStatorCur.getAppliedUpdateFrequency( )));
    }

    DataLogManager.log(String.format("%s: Initial position %.1f inches", getSubsystem( ), m_leftCurInches));

    // Simulation object initialization
    m_climberSim.Orientation = ChassisReference.CounterClockwise_Positive;

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

    if (m_climberValid)
    {
      BaseStatusSignal.refreshAll(m_leftPosition, m_rightPosition);
      m_leftCurInches = Conversions.rotationsToWinchInches(m_leftPosition.getValue( ), kRolloutRatio);
      m_rightCurInches = Conversions.rotationsToWinchInches(m_rightPosition.getValue( ), kRolloutRatio);
      if (m_leftCurInches < 0)
        setClimberPosition(0.0);
    }

    // Update dashboard
    m_leftCalibratedEntry.setBoolean((m_leftCalibrated));
    m_rightCalibratedEntry.setBoolean((m_rightCalibrated));
    m_leftInchesEntry.setDouble(m_leftCurInches);
    m_rightInchesEntry.setDouble(m_rightCurInches);
    m_targetInchesEntry.setDouble(m_targetInches);
    // m_leftCLoopErrorEntry.setDouble(m_targetInches - m_leftCurInches);
    // m_rightCLoopErrorEntry.setDouble(m_targetInches - m_rightCurInches);
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation

    // Set input motor voltage from the motor setting
    m_climberSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_elevSim.setInput(m_climberSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_elevSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_climberSim.setRawRotorPosition(
        Conversions.inchesToWinchRotations(Units.metersToInches(m_elevSim.getPositionMeters( )), kRolloutRatio));
    m_climberSim.setRotorVelocity(
        Conversions.inchesToWinchRotations(Units.metersToInches(m_elevSim.getVelocityMetersPerSecond( )), kRolloutRatio));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevSim.getCurrentDrawAmps( )));

    m_mechLigament.setLength(0.1 + Units.inchesToMeters(m_leftCurInches));
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Initialize dashboard widgets
    m_subsystemTab.add(kSubsystemName + "Mech", m_climberMech).withPosition(0, 2);

    ShuffleboardLayout cmdList = m_subsystemTab.getLayout("Commands", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 3)
        .withProperties(Map.of("Label position", "HIDDEN"));
    cmdList.add("ClRunExtended", getMoveToPositionCommand(this::getClimberFullyExtended));
    cmdList.add("ClRunChain", getMoveToPositionCommand(this::getClimberChainLevel));
    cmdList.add("ClRunClimbed", getMoveToPositionCommand(this::getClimberClimbed));
    cmdList.add("ClCalibrate", getCalibrateCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during mode changes
   */
  public void initialize( )
  {
    setVoltage(0.0, 0.0);
    m_leftCalibrated = false;
    m_rightCalibrated = false;

    m_leftCurInches = 0.0; // Allow calibration routine to run for up to this length
    m_targetInches = m_leftCurInches;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Inches: %.1f", getSubsystem( ), m_targetInches));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (m_climberValid)
    {
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_leftMotor, "ClimeberLeft");
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rightMotor, "ClimeberRight");
      m_leftMotor.clearStickyFaults( );
      m_rightMotor.clearStickyFaults( );
    }
    else
    {
      DataLogManager.log(String.format("%s: m_climberValid is FALSE!", getSubsystem( )));
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MANUAL MOVEMENT //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Move motors proportional to a joystick axis value
   * 
   * @param getAxis
   *          double supplier that returns desired joystick axis
   */
  private void moveWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
    boolean rangeLimited = false;
    ClimberMode newMode = ClimberMode.STOP;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_leftCurInches > kLengthMin))
      newMode = ClimberMode.DOWN;
    else if ((axisValue > 0.0) && (m_leftCurInches < kLengthMax))
      newMode = ClimberMode.UP;
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s now %.1f inches %s", getSubsystem( ), m_mode, m_leftCurInches,
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetInches = m_leftCurInches;

    setVoltage(axisValue * kManualSpeedVolts, axisValue * kManualSpeedVolts);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement
   * 
   * @param newLength
   *          distance to move
   * @param holdPosition
   *          hold previous position if true
   */
  private void moveToPositionInit(double newLength, boolean holdPosition)
  {
    m_mmMoveTimer.restart( );
    m_hardStopCounter = 0;

    if (!(m_leftCalibrated && m_rightCalibrated))
    {
      DataLogManager.log(String.format("%s: MM Position move target %.1f in - NOT CALIBRATED!", getSubsystem( ), m_targetInches));
      return;
    }

    if (holdPosition)
      newLength = m_leftCurInches;

    newLength = MathUtil.clamp(newLength, 0.25, kLengthMax);

    // Decide if a new position request
    if (holdPosition || newLength != m_targetInches || !MathUtil.isNear(newLength, m_leftCurInches, kToleranceInches))
    {
      // Validate the position request
      if (isMoveValid(newLength))
      {
        m_targetInches = newLength;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        setMMPosition(m_targetInches);

        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f inches (%.3f -> %.3f rot)", getSubsystem( ),
            m_leftCurInches, m_targetInches, Conversions.inchesToWinchRotations(m_leftCurInches, kRolloutRatio),
            Conversions.inchesToWinchRotations(m_targetInches, kRolloutRatio)));
      }
      else
        DataLogManager.log(String.format("%s: MM Position move target %.1f inches is OUT OF RANGE! [%.1f, %.1f rot]",
            getSubsystem( ), m_targetInches, kLengthMin, kLengthMax));
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Position already achieved - target %s inches", getSubsystem( ), m_targetInches));
    }
  }

  /****************************************************************************
   * 
   * Continuously update Motion Magic setpoint
   */
  private void moveToPositionExecute( )
  {}

  /****************************************************************************
   * 
   * Detect Motion Magic finished state
   * 
   * @return true when movement has completed
   */
  private boolean moveToPositionIsFinished(boolean hold)
  {
    boolean timedOut = m_mmMoveTimer.hasElapsed(kMMMoveTimeout);
    double error = m_targetInches - m_leftCurInches;
    boolean hittingHardStop = (m_targetInches <= 0.0) && (m_leftCurInches <= 1.0) && (m_hardStopCounter++ >= 10);

    setMMPosition(m_targetInches);

    if (hold)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceInches) || timedOut || hittingHardStop)
    {
      if (hittingHardStop)
        DataLogManager.log(String.format("%s - HITTINGHARDSTOP: %s", getSubsystem( ), hittingHardStop));
      if (!m_mmMoveIsFinished)
        DataLogManager.log(String.format("%s: MM Position move finished - Current inches: %.1f (error %.1f) - Time: %.3f sec %s",
            getSubsystem( ), m_leftCurInches, error, m_mmMoveTimer.get( ), (timedOut) ? "- TIMED OUT!" : ""));

      m_mmMoveIsFinished = true;
    }

    return m_mmMoveIsFinished;
  }

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  private void moveToPositionEnd( )
  {
    m_mmMoveTimer.stop( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CALIBRATION //////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize calibration movement
   */
  private void calibrateInit( )
  {
    DataLogManager.log(String.format("%s: Start up (%.1fV, %.1fV)", getSubsystem( ), kCalibrateSpeedVolts, kCalibrateSpeedVolts));
    m_leftCalibrated = false;
    m_rightCalibrated = false;
    m_calibrateTimer.restart( );
    // Reset the debouce filters
    m_leftStalled.calculate(false);
    m_rightStalled.calculate(false);
    setVoltage(kCalibrateSpeedVolts, kCalibrateSpeedVolts);
  }

  /****************************************************************************
   * 
   * Move climber down during calibration
   * 
   */
  private void calibrateExecute( )
  {}

  /****************************************************************************
   * 
   * Check for climber full down during calibration
   * 
   * @return true when command has completed
   */
  private boolean calibrateIsFinished( )
  {
    boolean leftCalibrated = m_leftStalled.calculate(m_leftStatorCur.getValue( ) > kCalibrateStallAmps);
    boolean rightCalibrated = m_rightStalled.calculate(m_rightStatorCur.getValue( ) > kCalibrateStallAmps);

    if (leftCalibrated && !m_leftCalibrated)
      DataLogManager.log(String.format("%s: Left stalled %s (right %s)", getSubsystem( ), leftCalibrated, rightCalibrated));
    if (rightCalibrated && !m_rightCalibrated)
      DataLogManager.log(String.format("%s: Right stalled %s (left %s)", getSubsystem( ), rightCalibrated, leftCalibrated));

    m_leftCalibrated = leftCalibrated;
    m_rightCalibrated = rightCalibrated;

    setVoltage((m_leftCalibrated) ? 0.0 : kCalibrateSpeedVolts, (m_rightCalibrated) ? 0.0 : kCalibrateSpeedVolts);

    return (m_leftCalibrated && m_rightCalibrated) || m_calibrateTimer.hasElapsed(kCalibrationTimeout);
  }

  /****************************************************************************
   * 
   * Wrap up calibration sequence
   */
  private void calibrateEnd( )
  {
    DataLogManager.log(String.format("%s: End - elapsed %.3f sec", getSubsystem( ), m_calibrateTimer.get( )));
    m_calibrateTimer.stop( );
    setClimberPosition(0.0);
    setVoltage(0.0, 0.0);
    m_targetInches = m_leftCurInches;
    m_leftCalibrated = true;
    m_rightCalibrated = true;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS ///////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set climber encoder position
   * 
   * @param inches
   *          height to set
   */
  private void setClimberPosition(double inches)
  {
    m_leftCurInches = inches;
    if (m_climberValid)
    {
      m_leftMotor.setPosition(Conversions.inchesToWinchRotations(inches, kRolloutRatio));
      m_rightMotor.setPosition(Conversions.inchesToWinchRotations(inches, kRolloutRatio));
    }
  }

  /****************************************************************************
   * 
   * Set climber motors to a known voltage
   * 
   * @param volts
   *          voltage to apply (0.0 is stopped)
   */
  private void setVoltage(double leftVolts, double rightVolts)
  {
    if (m_climberValid)
    {
      m_leftMotor.setControl(m_requestVolts.withOutput(leftVolts));
      m_rightMotor.setControl(m_requestVolts.withOutput(rightVolts));
    }
  }

  /****************************************************************************
   * 
   * Set Motion Magic setpoint based on passed length
   * 
   * @param targetInches
   *          distance to move
   */
  private void setMMPosition(double targetInches)
  {
    if (m_climberValid)
    {
      // y = mx + b, where 0 degrees is 0.0 climber and 90 degrees is 1/4 winch turn (the climber constant)
      m_leftMotor.setControl(m_mmRequestVolts.withPosition(Conversions.inchesToWinchRotations(targetInches, kRolloutRatio))
          .withFeedForward(m_totalArbFeedForward));
      m_rightMotor.setControl(m_mmRequestVolts.withPosition(Conversions.inchesToWinchRotations(targetInches, kRolloutRatio))
          .withFeedForward(m_totalArbFeedForward));
    }
  }

  /****************************************************************************
   * 
   * Validate requested climber move
   * 
   * @param inches
   *          distance requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double inches)
  {
    return (inches >= kLengthMin) && (inches <= kLengthMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current cliimber position
   * 
   * @return current climber position in inches
   */
  public double getClimberPosition( )
  {
    return m_leftCurInches;
  }

  /****************************************************************************
   * 
   * Return climber length for climbed state
   * 
   * @return climbed state length
   */
  public double getClimberClimbed( )
  {
    return kLengthClimbed;
  }

  /****************************************************************************
   * 
   * Return climber length for fully extended state
   * 
   * @return fully extended state length
   */
  public double getClimberFullyExtended( )
  {
    return kLengthFull;
  }

  /****************************************************************************
   * 
   * Return climber length for chain state
   * 
   * @return chain height state length
   */
  public double getClimberChainLevel( )
  {
    return kLengthChain;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create calibration command
   * 
   * @return continuous command that runs climber motors
   */
  public Command getCalibrateCommand( )
  {
    return new FunctionalCommand(       // Command with all phases declared
        ( ) -> calibrateInit( ),        // Init method
        ( ) -> calibrateExecute( ),     // Execute method
        interrupted -> calibrateEnd( ), // End method
        ( ) -> calibrateIsFinished( ),  // IsFinished method
        this                            // Subsytem required
    )                                   //
        .withName(kSubsystemName + "Calibrate");
  }

  /****************************************************************************
   * 
   * Create joystick manual move command
   * 
   * @param axis
   *          double supplier that provides the joystick axis value
   * @return continuous command that runs climber motors
   */
  public Command getJoystickCommand(DoubleSupplier axis)
  {
    return new RunCommand(              // Command that runs continuously
        ( ) -> moveWithJoystick(axis),  // Lambda method to call
        this                            // Subsystem required
    )                                   //
        .withName(kSubsystemName + "MoveWithJoystick");
  }

  /****************************************************************************
   * 
   * Create motion magic base command
   * 
   * @param position
   *          double supplier that provides the target distance
   * @param hold
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs climber motors
   */
  private Command getMMPositionCommand(DoubleSupplier position, boolean hold)
  {
    return new FunctionalCommand(                                 // Command with all phases declared
        ( ) -> moveToPositionInit(position.getAsDouble( ), hold), // Init method
        ( ) -> moveToPositionExecute( ),                          // Execute method
        interrupted -> moveToPositionEnd( ),                      // End method
        ( ) -> moveToPositionIsFinished(hold),                    // IsFinished method
        this                                                      // Subsytem required
    );
  }

  /****************************************************************************
   * 
   * Create motion magic move to position command
   * 
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getMoveToPositionCommand(DoubleSupplier position)
  {
    return getMMPositionCommand(position, false).withName(kSubsystemName + "MMMoveToPosition");
  }

  /****************************************************************************
   * 
   * Create motion magic hold position command
   * 
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getHoldPositionCommand(DoubleSupplier position)
  {
    return getMMPositionCommand(position, true).withName(kSubsystemName + "MMHoldPosition");
  }
}
