//
// Wrist Subystem - takes in Notes and delivers them to the other subsystems
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import frc.robot.Constants.INConsts.INRollerMode;
import frc.robot.Constants.Ports;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;

/****************************************************************************
 * 
 * Wrist subsystem to control the wrist roller and rotary mechanism and provide command factories
 */
public class Wrist extends SubsystemBase
{
  // Constants
  private static final String  kSubsystemName        = "Wrist";
  private static final boolean kRollerMotorInvert    = false;     // Motor direction for positive input

  private static final double  kRollerSpeedAcquire   = 0.5;
  private static final double  kRollerSpeedExpel     = -0.4;
  private static final double  kRollerSpeedToShooter = -1.0;
  private static final double  kRollerSpeedToFeeder  = -0.4;
  private static final double  kRollerSpeedHold      = 0.1;

  private static final double  kRotaryGearRatio      = 30.83;
  private static final double  kRotaryLengthMeters   = 0.3;       // Simulation
  private static final double  kRotaryWeightKg       = 4.0;       // Simulation
  private static final Voltage kRotaryManualVolts    = Volts.of(3.5);       // Motor voltage during manual operation (joystick)

  /** Rotary manual move parameters */
  private enum RotaryMode
  {
    INIT,    // Initialize rotary
    INBOARD, // Rotary moving into the robot
    STOPPED, // Rotary stop and hold position
    OUTBOARD // Rotary moving out of the robot
  }

  private static final double       kToleranceDegrees     = 3.0;      // PID tolerance in degrees
  private static final double       kMMDebounceTime       = 0.060;    // Seconds to debounce a final position check
  private static final double       kMMMoveTimeout        = 1.0;      // Seconds allowed for a Motion Magic movement
  private static final double       kNoteDebounceTime     = 0.045;    // Seconds to debounce detected note sensor

  // Rotary angles - Motion Magic move parameters
  //    Measured hardstops and pre-defined positions:
  //               hstop  retracted   handoff   deployed  hstop
  //      Comp     -177.3  -176.3     -124.7    24.9      25.8
  //      Practice -177.8  -176.8     -124.7    27.3      27.4
  private static final double       kRotaryAngleRetracted = Robot.isComp( ) ? -176.3 : -176.8;  // One degree from hardstops
  private static final double       kRotaryAngleHandoff   = Robot.isComp( ) ? -124.7 : -124.7;  //
  private static final double       kRotaryAngleDeployed  = Robot.isComp( ) ? 24.9 : 27.3;      //

  private static final double       kRotaryAngleMin       = kRotaryAngleRetracted - 3.0;
  private static final double       kRotaryAngleMax       = kRotaryAngleDeployed + 3.0;

  // Device objects
  private final WPI_TalonSRX        m_rollerMotor         = new WPI_TalonSRX(Ports.kCANID_WristRoller);
  private final TalonFX             m_rotaryMotor         = new TalonFX(Ports.kCANID_WristRotary);
  private final CANcoder            m_CANcoder            = new CANcoder(Ports.kCANID_WristCANcoder);
  // private final DigitalInput        m_noteInIntake        = new DigitalInput(Ports.kDIO0_NoteInIntake);

  // Alerts
  private final Alert               m_rollerAlert         =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert               m_rotaryAlert         =
      new Alert(String.format("%s: Rotary motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert               m_canCoderAlert       =
      new Alert(String.format("%s: CANcoder init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState     m_rotarySim           = m_rotaryMotor.getSimState( );
  private final CANcoderSimState    m_CANcoderSim         = m_CANcoder.getSimState( );
  private final SingleJointedArmSim m_armSim              = new SingleJointedArmSim(DCMotor.getFalcon500(1), kRotaryGearRatio,
      SingleJointedArmSim.estimateMOI(kRotaryLengthMeters, kRotaryWeightKg), kRotaryLengthMeters, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_rotaryMech          = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d m_mechLigament        = m_rotaryMech.getRoot("Rotary", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, 0.5, 0.0, 6, new Color8Bit(Color.kPurple)));

  // Status signals
  private final StatusSignal<Angle> m_rotaryPosition;  // Default 50Hz (20ms)
  private final StatusSignal<Angle> m_ccPosition;      // Default 100Hz (10ms)

  // Declare module variables

  // Roller variables
  private boolean                   m_rollerValid;        // Health indicator for motor 
  private Debouncer                 m_noteDebouncer       = new Debouncer(kNoteDebounceTime, DebounceType.kBoth);
  private boolean                   m_noteDetected;       // Detection state of note in rollers

  // Rotary variables
  private boolean                   m_rotaryValid;                // Health indicator for motor 
  private boolean                   m_canCoderValid;              // Health indicator for CANcoder 
  private double                    m_currentDegrees      = 0.0;  // Current angle in degrees
  private double                    m_targetDegrees       = 0.0;  // Target angle in degrees
  private double                    m_ccDegrees           = 0.0;  // CANcoder angle in degrees

  // Manual mode config parameters
  private VoltageOut                m_requestVolts        = new VoltageOut(Volts.of(0));
  private RotaryMode                m_rotaryMode          = RotaryMode.INIT;    // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_mmRequestVolts      = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                 m_mmWithinTolerance   = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                     m_mmMoveTimer         = new Timer( );       // Movement timer
  private boolean                   m_mmMoveIsFinished;           // Movement has completed (within tolerance)

  // Network tables publisher objects
  private DoublePublisher           m_rollSpeedPub;
  private DoublePublisher           m_rollSupCurPub;
  private DoublePublisher           m_rotDegreesPub;

  private DoublePublisher           m_ccDegreesPub;
  private DoublePublisher           m_targetDegreesPub;
  private BooleanPublisher          m_noteDetectedPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Wrist( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // // Roller motor init
    // m_rollerValid = PhoenixUtil5.getInstance( ).talonSRXInitialize(m_rollerMotor, kSubsystemName + "Roller",
    //     CTREConfigs5.intakeRollerConfig( ));
    // m_rollerMotor.setInverted(kRollerMotorInvert);
    // PhoenixUtil5.getInstance( ).talonSRXCheckError(m_rollerMotor, "setInverted");

    // // Initialize rotary motor and CANcoder objects
    // m_rotaryValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rotaryMotor, kSubsystemName + "Rotary",
    //     CTREConfigs6.intakeRotaryFXConfig(Units.degreesToRotations(kRotaryAngleMin), Units.degreesToRotations(kRotaryAngleMax),
    //         Ports.kCANID_WristCANcoder, kRotaryGearRatio));
    // m_canCoderValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_CANcoder, kSubsystemName + "Rotary",
    //     CTREConfigs6.intakeRotaryCancoderConfig( ));

    m_rollerAlert.set(!m_rollerValid);
    m_rotaryAlert.set(!m_rotaryValid);
    m_canCoderAlert.set(!m_canCoderValid);

    // Initialize status signal objects
    m_rotaryPosition = m_rotaryMotor.getPosition( );
    m_ccPosition = m_CANcoder.getAbsolutePosition( );

    // Initialize the climber status signals
    Double ccRotations = (m_canCoderValid) ? m_ccPosition.refresh( ).getValue( ).in(Rotations) : 0.0;
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));
    if (m_rotaryValid)
      m_rotaryMotor.setPosition(ccRotations);

    // Simulation object initialization
    m_rotarySim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_CANcoderSim.Orientation = ChassisReference.Clockwise_Positive;

    // Status signals
    m_rotaryPosition.setUpdateFrequency(50);

    StatusSignal<Current> m_rotarySupplyCur = m_rotaryMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Current> m_rotaryStatorCur = m_rotaryMotor.getStatorCurrent( ); // Default 4Hz (250ms)
    BaseStatusSignal.setUpdateFrequencyForAll(10, m_rotarySupplyCur, m_rotaryStatorCur);

    DataLogManager.log(
        String.format("%s: Update (Hz) rotaryPosition: %.1f rotarySupplyCur: %.1f rotaryStatorCur: %.1f canCoderPosition: %.1f",
            getSubsystem( ), m_rotaryPosition.getAppliedUpdateFrequency( ), m_rotarySupplyCur.getAppliedUpdateFrequency( ),
            m_rotaryStatorCur.getAppliedUpdateFrequency( ), m_ccPosition.getAppliedUpdateFrequency( )));

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

    BaseStatusSignal.refreshAll(m_rotaryPosition, m_ccPosition);
    m_currentDegrees = Units.rotationsToDegrees((m_rotaryValid) ? m_rotaryPosition.getValue( ).in(Rotations) : 0.0);
    m_ccDegrees = Units.rotationsToDegrees((m_canCoderValid) ? m_ccPosition.getValue( ).in(Rotations) : 0.0);
    // m_noteDetected = m_noteDebouncer.calculate(m_noteInIntake.get( ));

    // // Update network table publishers
    // m_rollSpeedPub.set(m_rollerMotor.get( ));
    // m_rollSupCurPub.set(m_rollerMotor.getSupplyCurrent( ));

    // m_ccDegreesPub.set(m_ccDegrees);
    // m_rotDegreesPub.set(m_currentDegrees);
    // m_targetDegreesPub.set(m_targetDegrees);
    // m_noteDetectedPub.set(m_noteDetected);
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
    m_rotarySim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_CANcoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInputVoltage(m_rotarySim.getMotorVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rotarySim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngleRads( ), kRotaryGearRatio));
    m_rotarySim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), kRotaryGearRatio));

    m_CANcoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads( )));
    m_CANcoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec( )));

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps( )));

    m_mechLigament.setAngle(-m_currentDegrees);
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("wrist");

    // Initialize network tables publishers
    m_rollSpeedPub = table.getDoubleTopic("rollSpeed").publish( );
    m_rollSupCurPub = table.getDoubleTopic("rollSupCur").publish( );

    m_ccDegreesPub = table.getDoubleTopic("ccDegrees").publish( );
    m_rotDegreesPub = table.getDoubleTopic("rotDegrees").publish( );
    m_noteDetectedPub = table.getBooleanTopic("noteDetected").publish( );
    m_targetDegreesPub = table.getDoubleTopic("targetDegrees").publish( );

    SmartDashboard.putData("INRotaryMech", m_rotaryMech);

    // Add commands
    // SmartDashboard.putData("InRollStop", getMoveToPositionCommand(INRollerMode.STOP, this::getCurrentPosition));

  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setRollerMode(INRollerMode.STOP);
    setRotaryStopped( );

    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    // PhoenixUtil5.getInstance( ).talonSRXPrintFaults(m_rollerMotor, kSubsystemName + "Roller");
    // PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rotaryMotor, kSubsystemName + "Rotary");
    // PhoenixUtil6.getInstance( ).cancoderPrintFaults(m_CANcoder, kSubsystemName + "CANcoder");

    // m_rollerMotor.clearStickyFaults( );
    // m_rotaryMotor.clearStickyFaults( );
    // m_CANcoder.clearStickyFaults( );
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
  private void moveRotaryWithJoystick(DoubleSupplier getAxis)
  {
    double axisValue = getAxis.getAsDouble( );
    boolean rangeLimited = false;
    RotaryMode newMode = RotaryMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentDegrees > kRotaryAngleMin))
      newMode = RotaryMode.INBOARD;
    else if ((axisValue > 0.0) && (m_currentDegrees < kRotaryAngleMax))
      newMode = RotaryMode.OUTBOARD;
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_rotaryMode)
    {
      m_rotaryMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getSubsystem( ), m_rotaryMode,
          getCurrentPosition( ), ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    m_rotaryMotor.setControl(m_requestVolts.withOutput(kRotaryManualVolts.times(axisValue)));
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement and control rollers
   * 
   * @param mode
   *          roller mode to apply
   * @param newAngle
   *          rotation to move
   * @param holdPosition
   *          hold previous position if true
   */
  public void moveToPositionInit(INRollerMode mode, double newAngle, boolean holdPosition)
  {
    setRollerMode(mode);
    m_mmMoveTimer.restart( );

    if (holdPosition)
      newAngle = getCurrentPosition( );

    // Decide if a new position request
    if (holdPosition || newAngle != m_targetDegrees || !MathUtil.isNear(newAngle, m_currentDegrees, kToleranceDegrees))
    {
      // Validate the position request
      if (isMoveValid(newAngle))
      {
        m_targetDegrees = newAngle;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        double targetRotations = Units.degreesToRotations(m_targetDegrees);
        m_rotaryMotor.setControl(m_mmRequestVolts.withPosition(targetRotations));
        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getSubsystem( ),
            m_currentDegrees, m_targetDegrees, Units.degreesToRotations(m_currentDegrees), targetRotations));
      }
      else
        DataLogManager.log(String.format("%s: MM Position move target %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]",
            getSubsystem( ), m_targetDegrees, kRotaryAngleMin, kRotaryAngleMax));
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Position already achieved -target %s degrees", getSubsystem( ), m_targetDegrees));
    }
  }

  /****************************************************************************
   * 
   * Continuously update Motion Magic setpoint
   */
  public void moveToPositionExecute( )
  {}

  /****************************************************************************
   * 
   * Detect Motion Magic finished state
   * 
   * @param holdPosition
   *          hold the current position
   * @return true when movement has completed
   */
  public boolean moveToPositionIsFinished(boolean holdPosition)
  {
    boolean timedOut = m_mmMoveTimer.hasElapsed(kMMMoveTimeout);
    double error = m_targetDegrees - m_currentDegrees;

    m_rotaryMotor.setControl(m_mmRequestVolts.withPosition(Units.degreesToRotations(m_targetDegrees)));

    if (holdPosition)
      return false;

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceDegrees) || timedOut)
    {
      if (!m_mmMoveIsFinished)
        DataLogManager
            .log(String.format("%s: MM Position move finished - Current degrees: %.1f (difference %.1f) - Time: %.3f sec %s",
                getSubsystem( ), m_currentDegrees, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));

      m_mmMoveIsFinished = true;
    }

    return m_mmMoveIsFinished;
  }

  /****************************************************************************
   * 
   * Wrap up a Motion Magic movement
   */
  public void moveToPositionEnd( )
  {
    m_mmMoveTimer.stop( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set roller speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  private void setRollerMode(INRollerMode mode)
  {

  }

  /****************************************************************************
   * 
   * Set rotary motor to stopped
   */
  private void setRotaryStopped( )
  {
    DataLogManager.log(String.format("%s: Rotary motor now STOPPED", getSubsystem( )));
    m_rotaryMotor.setControl(m_requestVolts.withOutput(0.0));
  }

  /****************************************************************************
   * 
   * Validate requested move
   * 
   * @param degrees
   *          angle requested
   * @return true if move is within range
   */
  private boolean isMoveValid(double degrees)
  {
    return (degrees >= kRotaryAngleMin) && (degrees <= kRotaryAngleMax);
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return current position
   * 
   * @return current rotary angle
   */
  public double getCurrentPosition( )
  {
    return m_currentDegrees;
  }

  /****************************************************************************
   * 
   * Return wrist angle for retracted state
   * 
   * @return retracted state angle
   */
  public double getIntakeRetracted( )
  {
    return kRotaryAngleRetracted;
  }

  /****************************************************************************
   * 
   * Return wrist angle for deployed state
   * 
   * @return deployed state angle
   */
  public double getWristDeployed( )
  {
    return kRotaryAngleDeployed;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create joystick manual move command
   * 
   * @param axis
   *          double supplier that provides the joystick axis value
   * @return continuous command that runs rotary motor
   */
  public Command getJoystickCommand(DoubleSupplier axis)
  {
    return new RunCommand(                    // Command that runs continuously
        ( ) -> moveRotaryWithJoystick(axis),  // Lambda method to call
        this                                  // Subsystem required
    )                                         //
        .withName(kSubsystemName + "MoveWithJoystick");
  }

  /****************************************************************************
   * 
   * Create motion magic base command
   * 
   * @param mode
   *          roller mode to apply
   * @param position
   *          double supplier that provides the target distance
   * @param holdPosition
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs climber motors
   */
  private Command getMMPositionCommand(INRollerMode mode, DoubleSupplier position, boolean holdPosition)
  {
    return new FunctionalCommand(                                               // Command with all phases declared
        ( ) -> moveToPositionInit(mode, position.getAsDouble( ), holdPosition), // Init method
        ( ) -> moveToPositionExecute( ),                                        // Execute method
        interrupted -> moveToPositionEnd( ),                                    // End method
        ( ) -> moveToPositionIsFinished(holdPosition),                          // IsFinished method
        this                                                                    // Subsytem required
    );
  }

  /****************************************************************************
   * 
   * Create motion magic move to position command
   * 
   * @param mode
   *          roller mode to apply
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getMoveToPositionCommand(INRollerMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, false).withName(kSubsystemName + "MMMoveToPosition");
  }

  /****************************************************************************
   * 
   * Create motion magic hold position command
   * 
   * @param mode
   *          roller mode to apply
   * @param position
   *          double supplier that provides the target distance value
   * @return continuous command that runs climber motors
   */
  public Command getHoldPositionCommand(INRollerMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, true).withName(kSubsystemName + "MMHoldPosition");
  }

}
