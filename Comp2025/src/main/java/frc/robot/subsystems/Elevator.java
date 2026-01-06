//
// Elevator Subystem - lifts the game pieces to score and robot to hang onto the cage
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private static final String  kSubsystemName          = "Elevator";
  private static final double  kGearRatio              = 8.627;           // Gear reduction
  private static final double  kSprocketDiameterInches = 1.751;           // Sprocket diameter in inches (22T * 0.25"/T / Pi)
  private static final double  kSprocketDiameterMeters = Units.inchesToMeters(kSprocketDiameterInches);
  private static final double  kRolloutRatioInches     = kSprocketDiameterInches * Math.PI / kGearRatio; // inches per shaft rotation
  private static final double  kRolloutRatioMeters     = Units.inchesToMeters(kRolloutRatioInches); // inches per shaft rotation
  private static final double  kCarriageMassKg         = Units.lbsToKilograms(32.0);     // Simulation
  private static final Voltage kManualSpeedVolts       = Volts.of(3.0); // Motor voltage during manual operation (joystick)

  private static final double  kToleranceInches        = 0.5;             // PID tolerance in inches
  private static final double  kMMDebounceTime         = 0.040;           // Seconds to debounce a final position check
  private static final double  kMMMoveTimeout          = 1.3;             // Seconds allowed for a Motion Magic movement
  // private static final Current kHardStopCurrentLimit   = Amps.of(100.0);

  // Elevator heights - Motion Magic move parameters
  private static final double  kHeightInchesMin        = 0.0;             // Minimum allowable height
  private static final double  kHeightInchesMax        = 30.5;            // Maximum allowable height

  private static final double  kMinDownHeight          = 0.1;             // Minimum height in inches commanded during down movements
  // private static final double  kMaxDownHeight          = 0.4;             // Maximum height in inches when down limit switch is closed

  private static final double  kHeightStowed           = 0.0;             // By definition - full down
  private static final double  kHeightCoralStation     = 0.0;             // By definition - at coral station

  private static final double  kHeightCoralL1          = 8.0;            // By definition - at L1 for scoring coral
  private static final double  kHeightCoralL2          = 8.0;             // By definition - at L2 for scoring coral
  private static final double  kHeightCoralL3          = 15.5;            // By definition - at L3 for scoring coral
  private static final double  kHeightCoralL4          = 28.25;            // By definition - at L4 for scoring coral

  private static final double  kHeightAlgaeProcessor   = 3.5;             // By definition - at scoring algae in processor
  private static final double  kHeightAlgaeL23         = 12.5;            // By definition - at L23 for taking algae
  private static final double  kHeightAlgaeL34         = 20.5;            // By definition - at L34 for taking algae
  private static final double  kHeightAlgaeNet         = 30.75;           // By definition - at scoring algae in net

  /** Elevator manual move parameters */
  public enum JoystickMode
  {
    INIT,   // Initialized state
    UP,     // Move upward
    STOP,   // Stopped
    DOWN    // Move downward
  }

  // Device objects
  private final TalonFX               m_leftMotor         = new TalonFX(Ports.kCANID_ElevatorLeft);
  private final TalonFX               m_rightMotor        = new TalonFX(Ports.kCANID_ElevatorRight);
  private final DigitalInput          m_elevatorDown      = new DigitalInput(Ports.kDIO0_ElevatorDown);

  // Alerts
  private final Alert                 m_leftAlert         =
      new Alert(String.format("%s: Left motor init fail!", getSubsystem( )), AlertType.kError);
  private final Alert                 m_rightAlert        =
      new Alert(String.format("%s: Right motor init fail!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState       m_leftMotorSim      = m_leftMotor.getSimState( );
  private final TalonFXSimState       m_rightMotorSim     = m_rightMotor.getSimState( );
  private final ElevatorSim           m_elevSim           =
      new ElevatorSim(DCMotor.getKrakenX60Foc(2), kGearRatio, kCarriageMassKg, kSprocketDiameterMeters / 2,
          Units.inchesToMeters(kHeightInchesMin - 0.1), Units.inchesToMeters(kHeightInchesMax + 0.1), true, 0.0);

  // Mechanism2d
  private final Mechanism2d           m_elevatorMech      = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d   m_mechLigament      = m_elevatorMech.getRoot("Linear", 0.5, 0.1).append(
      new MechanismLigament2d(kSubsystemName, 0.1 + Units.inchesToMeters(kHeightInchesMax), 90.0, 6, new Color8Bit(Color.kRed)));

  // Status signals for sensors
  private final StatusSignal<Angle>   m_leftPosition;     // Default 4Hz (250ms)
  private final StatusSignal<Current> m_leftStatorCur;    // Default 4Hz (250ms)
  private final StatusSignal<Angle>   m_rightPosition;    // Default 4Hz (250ms)
  private final StatusSignal<Current> m_rightStatorCur;   // Default 4Hz (250ms)

  // Declare module variables
  private boolean                     m_motorsValid;      // Health indicator for Kraken motors
  private double                      m_currentHeight     = 0.0; // Current height used for decisions
  private double                      m_goalHeight        = 0.0; // Goal height in inches
  private boolean                     m_calibrated        = false;

  // Manual mode config parameters
  private VoltageOut                  m_requestVolts      = new VoltageOut(Volts.of(0)).withEnableFOC(true);
  private JoystickMode                m_manualMode        = JoystickMode.INIT;      // Manual movement mode with joysticks

  // Motion Magic mode config parameters
  private MotionMagicVoltage          m_mmRequestVolts    = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private Debouncer                   m_mmWithinTolerance = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                       m_mmMoveTimer       = new Timer( );           // Movement timer
  private boolean                     m_mmMoveIsFinished  = true;                   // Movement has completed (within tolerance)
  private int                         m_mmHardStopCounter = 0;

  // Network tables publisher objects
  private DoublePublisher             m_leftHeightPub;
  private DoublePublisher             m_rightHeightPub;
  private DoublePublisher             m_currentHeightPub;
  private DoublePublisher             m_goalHeightPub;
  private BooleanPublisher            m_calibratedPub;
  private BooleanPublisher            m_isDownPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Elevator( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    double min = Conversions.rolloutToWinchRotations(kHeightInchesMin, kRolloutRatioInches);
    double max = Conversions.rolloutToWinchRotations(kHeightInchesMax, kRolloutRatioInches);
    TalonFXConfiguration cfg;

    // Initialize motor objects
    cfg = CTREConfigs6.elevatorFXConfig(true, min, max);
    boolean leftValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_leftMotor, kSubsystemName + "Left", cfg);
    cfg = CTREConfigs6.elevatorFXConfig(false, min, max);
    boolean rightValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rightMotor, kSubsystemName + "Right", cfg);
    m_motorsValid = leftValid && rightValid;

    m_leftAlert.set(!leftValid);
    m_rightAlert.set(!rightValid);

    // Initialize motor status signal objects
    m_leftPosition = m_leftMotor.getRotorPosition( );
    m_leftStatorCur = m_leftMotor.getStatorCurrent( );
    m_rightPosition = m_rightMotor.getRotorPosition( );
    m_rightStatorCur = m_rightMotor.getStatorCurrent( );

    // Initialize the motor status signals
    if (m_motorsValid)
    {
      setPosition(m_currentHeight);

      // Status signals
      BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftPosition, m_rightPosition, m_leftStatorCur, m_rightStatorCur);
      DataLogManager
          .log(String.format("%s: Update (Hz) leftPosition: %.1f rightPosition: %.1f leftStatorCur: %.1f rightStatorCur: %.1f",
              getSubsystem( ), m_leftPosition.getAppliedUpdateFrequency( ), m_rightPosition.getAppliedUpdateFrequency( ),
              m_leftStatorCur.getAppliedUpdateFrequency( ), m_rightStatorCur.getAppliedUpdateFrequency( )));
    }

    DataLogManager.log(String.format("%s: Initial position %.1f inches", getSubsystem( ), m_currentHeight));

    // Simulation object initialization
    m_leftMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    m_rightMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

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

    if (m_motorsValid)
    {
      BaseStatusSignal.refreshAll(m_leftPosition, m_rightPosition, m_leftStatorCur, m_rightStatorCur);
      double leftHeight = Conversions.rotationsToWinchRollout(m_leftPosition.getValue( ).in(Rotations), kRolloutRatioInches);
      double rightHeight = Conversions.rotationsToWinchRollout(m_rightPosition.getValue( ).in(Rotations), kRolloutRatioInches);
      m_leftHeightPub.set(leftHeight);
      m_rightHeightPub.set(rightHeight);
      m_currentHeight = leftHeight;
      m_currentHeightPub.set(m_currentHeight);

      // Calibrate if elevator is full down and not already calibrated
      boolean normalCalibrate = !m_calibrated && isDown( );
      // Reset motor positions if either left or right are significantly negative (probably restarted while still is up)
      boolean restartOutOfSync = DriverStation.isDisabled( ) && (leftHeight < -0.25 || rightHeight < -0.25);
      // Reset motor positions if full down, but do not change calibrate state (chain slip) -- set larger height to lower height // TODO
      // boolean fullDownOutOfSync = isDown( )
      //     && (!MathUtil.isNear(leftHeight, kHeightInchesMin, 0.25) || !MathUtil.isNear(rightHeight, kHeightInchesMin, 0.25));
      // Detect overcurrent when jammed // TODO
      // boolean overcurrent = isDown( ) && ((m_leftStatorCur.getValue( ).abs(Amps) > kHardStopCurrentLimit.in(Amps))
      //     || (m_rightStatorCur.getValue( ).abs(Amps) > kHardStopCurrentLimit.in(Amps)));

      // Handle height reset and calibration
      if (normalCalibrate || restartOutOfSync)
      {
        resetHeight(normalCalibrate);
      }
    }

    // Update network table publishers
    m_goalHeightPub.set(m_goalHeight);
    m_calibratedPub.set(m_calibrated);
    m_isDownPub.set(isDown( ));
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
    m_leftMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_rightMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_elevSim.setInput(m_leftMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_elevSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_leftMotorSim.setRawRotorPosition(Conversions.rolloutToWinchRotations(m_elevSim.getPositionMeters( ), kRolloutRatioMeters));
    m_leftMotorSim
        .setRotorVelocity(Conversions.rolloutToWinchRotations(m_elevSim.getVelocityMetersPerSecond( ), kRolloutRatioMeters));
    m_rightMotorSim.setRawRotorPosition(Conversions.rolloutToWinchRotations(m_elevSim.getPositionMeters( ), kRolloutRatioMeters));
    m_rightMotorSim
        .setRotorVelocity(Conversions.rolloutToWinchRotations(m_elevSim.getVelocityMetersPerSecond( ), kRolloutRatioMeters));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevSim.getCurrentDrawAmps( )));

    m_mechLigament.setLength(0.1 + Units.inchesToMeters(m_currentHeight));
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTable table = NetworkTableInstance.getDefault( ).getTable("elevator");

    // Initialize network tables publishers
    m_leftHeightPub = table.getDoubleTopic("leftInches").publish( );
    m_rightHeightPub = table.getDoubleTopic("rightInches").publish( );
    m_currentHeightPub = table.getDoubleTopic("currentInches").publish( );
    m_goalHeightPub = table.getDoubleTopic("goalInches").publish( );
    m_calibratedPub = table.getBooleanTopic("calibrated").publish( );
    m_isDownPub = table.getBooleanTopic("isDown").publish( );

    SmartDashboard.putData(kSubsystemName + "Mech", m_elevatorMech);

    // Add commands
    SmartDashboard.putData("ElRunStowed", getMoveToPositionCommand(this::getHeightStowed));
    SmartDashboard.putData("ElRunCoralStation", getMoveToPositionCommand(this::getHeightCoralStation));
    SmartDashboard.putData("ElRunCoralL1", getMoveToPositionCommand(this::getHeightCoralL1));
    SmartDashboard.putData("ElRunCoralL2", getMoveToPositionCommand(this::getHeightCoralL2));
    SmartDashboard.putData("ElRunCoralL3", getMoveToPositionCommand(this::getHeightCoralL3));
    SmartDashboard.putData("ElRunCoralL4", getMoveToPositionCommand(this::getHeightCoralL4));
    SmartDashboard.putData("ElRunAlgae23", getMoveToPositionCommand(this::getHeightAlgaeL23));
    SmartDashboard.putData("ElRunAlgae34", getMoveToPositionCommand(this::getHeightAlgaeL34));
    SmartDashboard.putData("ElRunNet", getMoveToPositionCommand(this::getHeightAlgaeNet));
    SmartDashboard.putData("ElRunProcessor", getMoveToPositionCommand(this::getHeightAlgaeProcessor));

    SmartDashboard.putData("ElCalibrate", getCalibrateHeightCommand( ));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setVoltage(Volts.of(0.0), Volts.of(0.0));

    m_goalHeight = m_currentHeight;
    DataLogManager.log(String.format("%s: Subsystem initialized! goal Inches: %.1f", getSubsystem( ), m_goalHeight));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (m_motorsValid)
    {
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_leftMotor, "ElevatorLeft");
      PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rightMotor, "ElevatorRight");
      m_leftMotor.clearStickyFaults( );
      m_rightMotor.clearStickyFaults( );
    }
    else
    {
      DataLogManager.log(String.format("%s: m_motorsValid is FALSE!", getSubsystem( )));
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MANUAL MOVEMENT //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  private final static Voltage kManualKG = Volts.of(0.325); // Elevator kG from tuning

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
    JoystickMode newMode = JoystickMode.STOP;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentHeight >= kHeightInchesMin))
    {
      newMode = JoystickMode.DOWN;
    }
    else if ((axisValue > 0.0) && (m_currentHeight <= kHeightInchesMax))
    {
      newMode = JoystickMode.UP;
    }
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_manualMode)
    {
      m_manualMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s now %.1f inches %s", getSubsystem( ), m_manualMode,
          m_currentHeight, ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_goalHeight = m_currentHeight;

    setVoltage(kManualSpeedVolts.times(axisValue).plus(kManualKG), kManualSpeedVolts.times(axisValue).plus(kManualKG));
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

    if (!(m_calibrated))
    {
      DataLogManager.log(String.format("%s: MM Position move goal %.1f in - NOT CALIBRATED!", getSubsystem( ), m_goalHeight));
      return;
    }

    if (holdPosition)
    {
      newHeight = m_currentHeight;
    }

    newHeight = MathUtil.clamp(newHeight, kMinDownHeight, kHeightInchesMax);

    // Decide if a new position request
    if (holdPosition || newHeight != m_goalHeight || !MathUtil.isNear(newHeight, m_currentHeight, kToleranceInches))
    {
      // Validate the position request
      if (isMoveValid(newHeight))
      {
        m_goalHeight = newHeight;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        setMMPosition(m_goalHeight);

        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f inches (%.3f -> %.3f rot)", getSubsystem( ),
            m_currentHeight, m_goalHeight, Conversions.rolloutToWinchRotations(m_currentHeight, kRolloutRatioInches),
            Conversions.rolloutToWinchRotations(m_goalHeight, kRolloutRatioInches)));
      }
      else
      {
        DataLogManager.log(String.format("%s: MM Position move goal %.1f inches is OUT OF RANGE! [%.1f, %.1f rot]",
            getSubsystem( ), m_goalHeight, kHeightInchesMin, kHeightInchesMax));
      }
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Position already achieved - goal %s inches", getSubsystem( ), m_goalHeight));
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
    double error = m_goalHeight - m_currentHeight;
    boolean hittingHardStop = (m_goalHeight <= 0.0) && (m_currentHeight <= 1.0) && (m_mmHardStopCounter++ >= 10);

    setMMPosition(m_goalHeight);

    if (holdPosition)
    {
      return false;
    }

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceInches) || timedOut || hittingHardStop)
    {
      if (hittingHardStop)
      {
        DataLogManager.log(String.format("%s - hittingHardStop: %s", getSubsystem( ), hittingHardStop));
      }

      if (!m_mmMoveIsFinished)
      {
        DataLogManager
            .log(String.format("%s: MM Position move finished - Current inches: %.1f (difference %.1f) - Time: %.3f sec %s",
                getSubsystem( ), m_currentHeight, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));
      }

      SmartDashboard.putNumber("ELMoveTime", m_mmMoveTimer.get( ) - kMMDebounceTime);

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
  ///////////////////////// PRIVATE HELPERS ///////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set elevator encoder position
   * 
   * @param inches
   *          height to set
   */
  private void setPosition(double inches)
  {
    m_currentHeight = inches;
    if (m_motorsValid)
    {
      double rotations = Conversions.rolloutToWinchRotations(inches, kRolloutRatioInches);
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
    if (m_motorsValid)
    {
      m_leftMotor.setControl(m_requestVolts.withOutput(leftVolts));
      m_rightMotor.setControl(m_requestVolts.withOutput(rightVolts));
    }
  }

  /****************************************************************************
   * 
   * Set Motion Magic setpoint based on passed height
   * 
   * @param goalInches
   *          distance to move
   */
  private void setMMPosition(double goalInches)
  {
    if (m_motorsValid)
    {
      double rotations = Conversions.rolloutToWinchRotations(goalInches, kRolloutRatioInches);
      m_leftMotor.setControl(m_mmRequestVolts.withPosition(rotations));
      m_rightMotor.setControl(m_mmRequestVolts.withPosition(rotations));
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
    return (inches >= kHeightInchesMin) && (inches <= kHeightInchesMax);
  }

  /****************************************************************************
   * 
   * Check limit switch for full down position
   * 
   */
  private boolean isDown( )
  {
    return !m_elevatorDown.get( );
  }

  /****************************************************************************
   * 
   * Calibrate height
   * 
   */
  private void resetHeight(boolean calibrate)
  {
    setPosition(0);
    DataLogManager.log(String.format("%s: Subsystem calibrated! Height Inches: %.1f", getSubsystem( ), m_currentHeight));
    if (calibrate)
    {
      m_calibrated = true;
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current elevator position
   * 
   * @return current position in inches
   */
  public double getCurrentHeight( )
  {
    return m_currentHeight;
  }

  /****************************************************************************
   * 
   * Return elevator height for stowed state
   * 
   * @return stowed state height
   */
  public double getHeightStowed( )
  {
    return kHeightStowed;
  }

  /****************************************************************************
   * 
   * Return elevator height for coral station intake state
   * 
   * @return coral station intake state height
   */
  public double getHeightCoralStation( )
  {
    return kHeightCoralStation;
  }

  /****************************************************************************
   * 
   * Return elevator height for coral L1 scoring state
   * 
   * @return coral L1 scoring state height
   */
  public double getHeightCoralL1( )
  {
    return kHeightCoralL1;
  }

  /****************************************************************************
   * 
   * Return elevator height for coral L2 scoring state
   * 
   * @return coral L2 scoring state height
   */
  public double getHeightCoralL2( )
  {
    return kHeightCoralL2;
  }

  /****************************************************************************
   * 
   * Return elevator height for coral L3 scoring state
   * 
   * @return coral L3 scoring state height
   */
  public double getHeightCoralL3( )
  {
    return kHeightCoralL3;
  }

  /****************************************************************************
   * 
   * Return elevator height for coral L4 scoring state
   * 
   * @return coral L4 scoring state height
   */
  public double getHeightCoralL4( )
  {
    return kHeightCoralL4;
  }

  /****************************************************************************
   * 
   * Return elevator height for Algae L23 scoring state
   * 
   * @return algae L23 scoring height
   */
  public double getHeightAlgaeL23( )
  {
    return kHeightAlgaeL23;
  }

  /****************************************************************************
   * 
   * Return elevator height for Algae L34 scoring state
   * 
   * @return algae L34 scoring height
   */
  public double getHeightAlgaeL34( )
  {
    return kHeightAlgaeL34;
  }

  /****************************************************************************
   * 
   * Return elevator height for Algae Net scoring state
   * 
   * @return algae Net scoring height
   */
  public double getHeightAlgaeNet( )
  {
    return kHeightAlgaeNet;
  }

  /****************************************************************************
   * 
   * Return elevator height for Algae Processor scoring state
   * 
   * @return algae Processor scoring height
   */
  public double getHeightAlgaeProcessor( )
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
   * @return instant command to set calibrated state from dashboard
   */
  public Command getCalibrateHeightCommand( )
  {
    return new InstantCommand(          // Command with init only phase declared
        ( ) -> resetHeight(true),       // Init method
        this                            // Subsytem required
    )                                   //
        .ignoringDisable(true) //
        .withName(kSubsystemName + "CalibrateHeight");
  }

  /****************************************************************************
   * 
   * Create joystick manual move command
   * 
   * @param axis
   *          double supplier that provides the joystick axis value
   * @return continuous command that runs elevator motors using joystick
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
   *          double supplier that provides the goal distance
   * @param holdPosition
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs elevator motors to a position (Motion Magic)
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
   *          double supplier that provides the goal distance value
   * @return continuous command that runs elevator motors to a position (Motion Magic)
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
   *          double supplier that provides the goal distance value
   * @return continuous command that holds elevator motors in a position (Motion Magic)
   */
  public Command getHoldPositionCommand(DoubleSupplier position)
  {
    return getMMPositionCommand(position, true).withName(kSubsystemName + "MMHoldPosition");
  }
}
