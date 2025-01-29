//
// Elevator Subystem - lifts the robot to hang onto the chain
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 * Elevator subsystem class
 */
public class Elevator extends SubsystemBase
{
  // Constants
  private static final String  kSubsystemName        = "Elevator";
  private static final double  kGearRatio            = 16.0;            // Gear reduction
  private static final double  kElevatorHeightMeters = 0.5;             // Simulation
  private static final double  kCarriageMassKg       = 2.0;             // Simulation
  private static final double  kDrumDiameterInches   = 1.375;           // Drum diameter in inches
  private static final double  kDrumRadiusMeters     = Units.inchesToMeters(kDrumDiameterInches) / 2;
  private static final double  kRolloutRatio         = kDrumDiameterInches * Math.PI / kGearRatio; // inches per shaft rotation
  private static final Voltage kCalibrateSpeedVolts  = Volts.of(-1.0);  // Motor voltage during calibration
  private static final Voltage kManualSpeedVolts     = Volts.of(3.0); // Motor voltage during manual operation (joystick)
  private static final double  kCalibrateStallAmps   = 25.0;            // Motor amps during calibration stall
  private static final double  kCalibrateStallTime   = 0.100;           // Seconds of stall before calibrating
  private static final double  kCalibrationTimeout   = 3.0;             // Max calibration time

  private static final double  kToleranceInches      = 0.5;             // Elevator PID tolerance in inches
  private static final double  kMMDebounceTime       = 0.060;           // Seconds to debounce a final position check
  private static final double  kMMMoveTimeout        = 4.0;             // Seconds allowed for a Motion Magic movement

  // Elevator lengths - Motion Magic config parameters                  // TODO: define desired elevator heights for 2025
  private static final double  kHeightStowed         = 0.0;             // By definition - Elevator full down
  private static final double  kHeightCoralStation   = 0.0;             // By definition - Elevator at coral station
  private static final double  kHeightCoralL1        = 10.0;            // By definition - Elevator at L1 for scoring coral

  private static final double  kHeightCoralL2        = 0.0;             // By definition - Elevator at L2 for scoring coral
  private static final double  kHeightCoralL3        = 15.0;             // By definition - Elevator at L3 for scoring coral
  private static final double  kHeightCoralL4        = 20.0;            // By definition - Elevator at L4 for scoring coral

  private static final double  kHeightAlgaeL23       = 0.0;             // By definition - Elevator at L23 for taking algae
  private static final double  kHeightAlgaeL34       = 0.0;             // By definition - Elevator at L34 for taking algae
  private static final double  kHeightAlgaeNet       = 0.0;             // By definition - Elevator at L34 for scoring algae in net
  private static final double  kHeightAlgaeProcessor = 0.0;             // By definition - Elevator at scoring algae in processor

  private static final double  kHeightMin            = 0.0;             // Elevator minimum allowable length
  private static final double  kHeightMax            = 30.0;            // Elevator maximum allowable length (2" beyond high length)

  /** Elevator manual move parameters */
  private enum ElevatorMode
  {
    INIT,   // Initialize elevator
    UP,     // Elevator move upward
    STOP,   // Elevator stop
    DOWN    // Elevator move downward
  }

  // Device objects
  private final TalonFX               m_leftMotor         = new TalonFX(Ports.kCANID_ElevatorLeft);
  private final TalonFX               m_rightMotor        = new TalonFX(Ports.kCANID_ElevatorRight);
  private final DigitalInput          m_LimitSwitch       = new DigitalInput(Ports.kLimitSwitch);

  // Alerts
  private final Alert                 m_leftAlert         =
      new Alert(String.format("%s: Left motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                 m_rightAlert        =
      new Alert(String.format("%s: Right motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState       m_elevatorSim       = m_leftMotor.getSimState( );
  private final ElevatorSim           m_elevSim           = new ElevatorSim(DCMotor.getFalcon500(1), kGearRatio, kCarriageMassKg,
      kDrumRadiusMeters, -kHeightMax, kHeightMax, false, 0.0);

  // Mechanism2d
  private final Mechanism2d           m_elevatorMech      = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d   m_mechLigament      = m_elevatorMech.getRoot("Linear", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, kElevatorHeightMeters, 0.0, 6, new Color8Bit(Color.kRed)));

  // CTRE Status signals for sensors
  private final StatusSignal<Angle>   m_leftPosition;     // Default 4Hz (250ms)
  private final StatusSignal<Current> m_leftSupplyCur;    // Default 4Hz (250ms)
  private final StatusSignal<Current> m_leftStatorCur;    // Default 4Hz (250ms)
  private final StatusSignal<Angle>   m_rightPosition;    // Default 4Hz (250ms)
  private final StatusSignal<Current> m_rightSupplyCur;   // Default 4Hz (250ms)
  private final StatusSignal<Current> m_rightStatorCur;   // Default 4Hz (250ms)

  // Declare module variables
  private boolean                     m_elevatorValid;    // Health indicator for Falcon
  private double                      m_leftHeight        = 0.0; // Current length in inches on left (default) side
  private double                      m_rightHeight       = 0.0; // Current length in inches on right side
  private double                      m_targetHeight      = 0.0; // Target length in inches

  // Calibration variables
  private Timer                       m_calibrateTimer    = new Timer( );
  private Debouncer                   m_leftStalled       = new Debouncer(kCalibrateStallTime, DebounceType.kRising);
  private Debouncer                   m_rightStalled      = new Debouncer(kCalibrateStallTime, DebounceType.kRising);
  private boolean                     m_leftCalibrated    = false;
  private boolean                     m_rightCalibrated   = false;

  // Manual mode config parameters
  private VoltageOut                  m_requestVolts      = new VoltageOut(Volts.of(0));
  private ElevatorMode                m_mode              = ElevatorMode.INIT;      // Manual movement mode with joysticks

  // Motion Magic mode config parameters
  private MotionMagicVoltage          m_mmRequestVolts    = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                   m_mmWithinTolerance = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                       m_mmMoveTimer       = new Timer( );           // Movement timer
  private Voltage                     m_mmArbFeedForward  = Volts.of(0);  // Arbitrary feedforward added to counteract gravity
  private int                         m_mmHardStopCounter = 0;
  private boolean                     m_mmMoveIsFinished  = true;                   // Movement has completed (within tolerance)

  // Network tables publisher objects
  private BooleanPublisher            m_leftCalibratedPub;
  private BooleanPublisher            m_rightCalibratedPub;
  private DoublePublisher             m_leftHeightPub;
  private DoublePublisher             m_rightHeightPub;
  private DoublePublisher             m_targetHeightPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Elevator( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Initialize elevator motor objects
    boolean leftValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_leftMotor, kSubsystemName + "Left",
        CTREConfigs6.elevatorFXConfig(true, Units.degreesToRotations(kHeightMin), Units.degreesToRotations(kHeightMax)));
    boolean rightValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rightMotor, kSubsystemName + "Right",
        CTREConfigs6.elevatorFXConfig(false, Units.degreesToRotations(kHeightMin), Units.degreesToRotations(kHeightMax)));
    m_elevatorValid = leftValid && rightValid;

    m_leftAlert.set(!leftValid);
    m_rightAlert.set(!rightValid);

    // Initialize status signal objects
    m_leftPosition = m_leftMotor.getRotorPosition( );
    m_leftSupplyCur = m_leftMotor.getSupplyCurrent( );
    m_leftStatorCur = m_leftMotor.getStatorCurrent( );
    m_rightPosition = m_rightMotor.getRotorPosition( );
    m_rightSupplyCur = m_rightMotor.getSupplyCurrent( );
    m_rightStatorCur = m_rightMotor.getStatorCurrent( );

    // Initialize the elevator status signals
    if (m_elevatorValid)
    {
      setElevatorPosition(m_leftHeight);

      // Status signals
      BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftPosition, m_rightPosition);

      DataLogManager.log(String.format(
          "%s: Update (Hz) leftPosition: %.1f rightPosition: %.1f leftSupplyCur: %.1f leftStatorCur: %.1f rightSupplyCur: %.1f rightStatorCur: %.1f",
          getSubsystem( ), m_leftPosition.getAppliedUpdateFrequency( ), m_rightPosition.getAppliedUpdateFrequency( ),
          m_leftSupplyCur.getAppliedUpdateFrequency( ), m_leftStatorCur.getAppliedUpdateFrequency( ),
          m_rightSupplyCur.getAppliedUpdateFrequency( ), m_rightStatorCur.getAppliedUpdateFrequency( )));
    }

    DataLogManager.log(String.format("%s: Initial position %.1f inches", getSubsystem( ), m_leftHeight));

    // Simulation object initialization
    m_elevatorSim.Orientation = ChassisReference.CounterClockwise_Positive;

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

    if (m_elevatorValid)
    {
      BaseStatusSignal.refreshAll(m_leftPosition, m_rightPosition);
      m_leftHeight = Conversions.rotationsToWinchInches(m_leftPosition.getValue( ).in(Rotations), kRolloutRatio);
      m_rightHeight = Conversions.rotationsToWinchInches(m_rightPosition.getValue( ).in(Rotations), kRolloutRatio);
      if (m_leftHeight < 0)
        setElevatorPosition(0.0);
    }

    // Update network table publishers
    m_leftCalibratedPub.set((m_leftCalibrated));
    m_rightCalibratedPub.set((m_rightCalibrated));
    m_leftHeightPub.set(m_leftHeight);
    m_rightHeightPub.set(m_rightHeight);
    m_targetHeightPub.set(m_targetHeight);
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
    m_elevatorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_elevSim.setInput(m_elevatorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_elevSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_elevatorSim.setRawRotorPosition(
        Conversions.inchesToWinchRotations(Units.metersToInches(m_elevSim.getPositionMeters( )), kRolloutRatio));
    m_elevatorSim.setRotorVelocity(
        Conversions.inchesToWinchRotations(Units.metersToInches(m_elevSim.getVelocityMetersPerSecond( )), kRolloutRatio));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevSim.getCurrentDrawAmps( )));

    m_mechLigament.setLength(0.1 + Units.inchesToMeters(m_leftHeight));
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("elevator");

    // Initialize network tables publishers
    m_leftCalibratedPub = table.getBooleanTopic("leftCalibrated").publish( );
    m_rightCalibratedPub = table.getBooleanTopic("rightCalibrated").publish( );
    m_leftHeightPub = table.getDoubleTopic("leftInches").publish( );
    m_rightHeightPub = table.getDoubleTopic("rightInches").publish( );
    m_targetHeightPub = table.getDoubleTopic("targetInches").publish( );

    SmartDashboard.putData(kSubsystemName + "Mech", m_elevatorMech);

    // Add commands
    SmartDashboard.putData("ClRunStowed", getMoveToPositionCommand(this::getElevatorStowed));
    SmartDashboard.putData("ClRunCoralL1", getMoveToPositionCommand(this::getElevatorCoralL1));
    SmartDashboard.putData("ClCalibrate", getCalibrateCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setVoltage(Volts.of(0.0), Volts.of(0.0));
    m_leftCalibrated = false;
    m_rightCalibrated = false;

    m_leftHeight = 0.0; // Allow calibration routine to run for up to this length
    m_targetHeight = m_leftHeight;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Inches: %.1f", getSubsystem( ), m_targetHeight));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (m_elevatorValid)
    {
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_leftMotor, "ElevatorLeft");
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rightMotor, "ElevatorRight");
      m_leftMotor.clearStickyFaults( );
      m_rightMotor.clearStickyFaults( );
    }
    else
    {
      DataLogManager.log(String.format("%s: m_elevatorValid is FALSE!", getSubsystem( )));
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
    ElevatorMode newMode = ElevatorMode.STOP;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_leftHeight > kHeightMin))
      newMode = ElevatorMode.DOWN;
    else if ((axisValue > 0.0) && (m_leftHeight < kHeightMax))
      newMode = ElevatorMode.UP;
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_mode)
    {
      m_mode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s now %.1f inches %s", getSubsystem( ), m_mode, m_leftHeight,
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetHeight = m_leftHeight;

    setVoltage(kManualSpeedVolts.times(axisValue), kManualSpeedVolts.times(axisValue));
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement
   * 
   * @param newHeight
   *          distance to move
   * @param holdPosition
   *          hold previous position if true
   */
  private void moveToPositionInit(double newHeight, boolean holdPosition)
  {
    m_mmMoveTimer.restart( );
    m_mmHardStopCounter = 0;

    if (!(m_leftCalibrated && m_rightCalibrated))
    {
      DataLogManager.log(String.format("%s: MM Position move target %.1f in - NOT CALIBRATED!", getSubsystem( ), m_targetHeight));
      return;
    }

    if (holdPosition)
      newHeight = m_leftHeight;

    newHeight = MathUtil.clamp(newHeight, 0.25, kHeightMax);

    // Decide if a new position request
    if (holdPosition || newHeight != m_targetHeight || !MathUtil.isNear(newHeight, m_leftHeight, kToleranceInches))
    {
      // Validate the position request
      if (isMoveValid(newHeight))
      {
        m_targetHeight = newHeight;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        setMMPosition(m_targetHeight);

        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f inches (%.3f -> %.3f rot)", getSubsystem( ),
            m_leftHeight, m_targetHeight, Conversions.inchesToWinchRotations(m_leftHeight, kRolloutRatio),
            Conversions.inchesToWinchRotations(m_targetHeight, kRolloutRatio)));
      }
      else
        DataLogManager.log(String.format("%s: MM Position move target %.1f inches is OUT OF RANGE! [%.1f, %.1f rot]",
            getSubsystem( ), m_targetHeight, kHeightMin, kHeightMax));
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Position already achieved - target %s inches", getSubsystem( ), m_targetHeight));
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
   * @param holdPosition
   *          hold the current position
   * @return true when movement has completed
   */
  private boolean moveToPositionIsFinished(boolean holdPosition)
  {
    boolean timedOut = m_mmMoveTimer.hasElapsed(kMMMoveTimeout);
    double error = m_targetHeight - m_leftHeight;
    boolean hittingHardStop = (m_targetHeight <= 0.0) && (m_leftHeight <= 1.0) && (m_mmHardStopCounter++ >= 10);

    setMMPosition(m_targetHeight);

    if (holdPosition)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceInches) || timedOut || hittingHardStop)
    {
      if (hittingHardStop)
        DataLogManager.log(String.format("%s - hittingHardStop: %s", getSubsystem( ), hittingHardStop));
      if (!m_mmMoveIsFinished)
        DataLogManager
            .log(String.format("%s: MM Position move finished - Current inches: %.1f (difference %.1f) - Time: %.3f sec %s",
                getSubsystem( ), m_leftHeight, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));

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
    // Reset the calibration state, time, debounce filters, and motor setting
    DataLogManager.log(String.format("%s: Start up (%s, %s)", getSubsystem( ), kCalibrateSpeedVolts.toString( ),
        kCalibrateSpeedVolts.toString( )));
    m_leftCalibrated = false;
    m_rightCalibrated = false;
    m_calibrateTimer.restart( );
    m_leftStalled.calculate(false);
    m_rightStalled.calculate(false);
    setVoltage(kCalibrateSpeedVolts, kCalibrateSpeedVolts);
  }

  /****************************************************************************
   * 
   * Move elevator down during calibration
   */
  private void calibrateExecute( )
  {}

  /****************************************************************************
   * 
   * Check for elevator full down during calibration
   * 
   * @return true when command has completed
   */
  private boolean calibrateIsFinished( )
  {
    boolean leftCalibrated = m_leftStalled.calculate(m_leftStatorCur.getValue( ).in(Amps) > kCalibrateStallAmps);
    boolean rightCalibrated = m_rightStalled.calculate(m_rightStatorCur.getValue( ).in(Amps) > kCalibrateStallAmps);

    if (leftCalibrated && !m_leftCalibrated)
      DataLogManager.log(String.format("%s: Left stalled %s (right %s)", getSubsystem( ), leftCalibrated, rightCalibrated));
    if (rightCalibrated && !m_rightCalibrated)
      DataLogManager.log(String.format("%s: Right stalled %s (left %s)", getSubsystem( ), rightCalibrated, leftCalibrated));

    m_leftCalibrated = leftCalibrated;
    m_rightCalibrated = rightCalibrated;

    setVoltage((m_leftCalibrated) ? Volts.of(0.0) : kCalibrateSpeedVolts,
        (m_rightCalibrated) ? Volts.of(0.0) : kCalibrateSpeedVolts);

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
    setElevatorPosition(0.0);
    setVoltage(Volts.of(0.0), Volts.of(0.0));
    m_targetHeight = m_leftHeight;
    m_leftCalibrated = true;
    m_rightCalibrated = true;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS ///////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set elevator encoder position
   * 
   * @param inches
   *          height to set
   */
  private void setElevatorPosition(double inches)
  {
    m_leftHeight = inches;
    if (m_elevatorValid)
    {
      double rotations = Conversions.inchesToWinchRotations(inches, kRolloutRatio);
      m_leftMotor.setPosition(rotations);
      m_rightMotor.setPosition(rotations);
    }
  }

  /****************************************************************************
   * 
   * Set elevator motors to a known voltage
   * 
   * @param leftVolts
   *          voltage to apply to left motor (0.0 is stopped)
   * @param rightVolts
   *          voltage to apply to right motor (0.0 is stopped)
   */
  private void setVoltage(Voltage leftVolts, Voltage rightVolts)
  {
    if (m_elevatorValid)
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
    if (m_elevatorValid)
    {
      // y = mx + b, where 0 degrees is 0.0 elevator and 90 degrees is 1/4 winch turn (the elevator constant)
      double position = Conversions.inchesToWinchRotations(targetInches, kRolloutRatio);
      m_leftMotor.setControl(m_mmRequestVolts.withPosition(position).withFeedForward(m_mmArbFeedForward));
      m_rightMotor.setControl(m_mmRequestVolts.withPosition(position).withFeedForward(m_mmArbFeedForward));
    }
  }

  /****************************************************************************
   * 
   * Validate requested elevator move
   * 
   * @param inches
   *          distance requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double inches)
  {
    return (inches >= kHeightMin) && (inches <= kHeightMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current cliimber position
   * 
   * @return current elevator position in inches
   */
  public double getElevatorPosition( )
  {
    return m_leftHeight;
  }

  /****************************************************************************
   * 
   * Return elevator length for stowed state
   * 
   * @return elevator stowed state length
   */
  public double getElevatorStowed( )
  {
    return kHeightStowed;
  }

  /****************************************************************************
   * 
   * Return elevator length for coral L1 scoring state
   * 
   * @return elevator coral L1 scoring state length
   */
  public double getElevatorCoralL1( )
  {
    return kHeightCoralL1;
  }

  /****************************************************************************
   * 
   * Return elevator length for coral L2 scoring state
   * 
   * @return elevator coral L2 scoring state length
   */
  public double getElevatorCoralL2( )
  {
    return kHeightCoralL2;
  }

  /****************************************************************************
   * 
   * Return elevator length for coral L3 scoring state
   * 
   * @return elevator coral L3 scoring state length
   */
  public double getElevatorCoralL3( )
  {
    return kHeightCoralL3;
  }

  /****************************************************************************
   * 
   * Return elevator length for coral L4 scoring state
   * 
   * @return elevator coral L4 scoring state length
   */
  public double getElevatorCoralL4( )
  {
    return kHeightCoralL4;
  }

  /****************************************************************************
   * 
   * Return elevator length for coral station intake state
   * 
   * @return elevator coral station intake state length
   */
  public double getkHeightCoralStation( )
  {
    return kHeightCoralStation;
  }

  /****************************************************************************
   * 
   * Return elevator length for Algae L23 scoring state
   * 
   * @return elevator Algae L23 scoring length
   */
  public double getkHeightAlgaeL23( )
  {
    return kHeightAlgaeL23;
  }

  /****************************************************************************
   * 
   * Return elevator length for Algae L34 scoring state
   * 
   * @return elevator Algae L34 scoring length
   */
  public double getkHeightAlgaeL34( )
  {
    return kHeightAlgaeL34;
  }

  /****************************************************************************
   * 
   * Return elevator length for Algae Net scoring state
   * 
   * @return elevator Algae Net scoring length
   */
  public double getkHeightAlgaeLNet( )
  {
    return kHeightAlgaeNet;
  }

  /****************************************************************************
   * 
   * Return elevator length for Algae Processor scoring state
   * 
   * @return elevator Algae Processor scoring length
   */
  public double getkHeightAlgaeProcessor( )
  {
    return kHeightAlgaeProcessor;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create calibration command
   * 
   * @return continuous command that runs elevator motors
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
   * @return continuous command that runs elevator motors
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
   * @param holdPosition
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs elevator motors
   */
  private Command getMMPositionCommand(DoubleSupplier position, boolean holdPosition)
  {
    return new FunctionalCommand(                                         // Command with all phases declared
        ( ) -> moveToPositionInit(position.getAsDouble( ), holdPosition), // Init method
        ( ) -> moveToPositionExecute( ),                                  // Execute method
        interrupted -> moveToPositionEnd( ),                              // End method
        ( ) -> moveToPositionIsFinished(holdPosition),                    // IsFinished method
        this                                                              // Subsytem required
    );
  }

  /****************************************************************************
   * 
   * Create motion magic move to position command
   * 
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs elevator motors
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
   * @return continuous command that runs elevator motors
   */
  public Command getHoldPositionCommand(DoubleSupplier position)
  {
    return getMMPositionCommand(position, true).withName(kSubsystemName + "MMHoldPosition");
  }
}
