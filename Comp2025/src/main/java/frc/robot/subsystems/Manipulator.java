//
// Manipulator Subystem - takes in Coral and Algae and delivers them to Reef, Net, and Processor
//
// The manipulator is composed of two motorized mechanisms: wrist rotary joint and a claw roller.
// The wrist rotary joint uses an external CANcoder for measuring rotation.
// The claw roller has a limit switch to detect coral and algae
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.CANrangeSimState;
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
import frc.robot.Constants.Ports;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Robot;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Manipulator subsystem to control the wrist rotary and claw roller mechanisms and provide command
 * factories
 */
public class Manipulator extends SubsystemBase
{
  // Constants
  private static final String  kSubsystemName       = "Manipulator";

  private static final double  kAlgaeSpeedAcquire   = 0.5;
  private static final double  kAlgaeSpeedExpel     = -0.4;
  private static final double  kAlgaeSpeedShoot     = -1.0;
  private static final double  kAlgaeSpeedProcessor = -0.4;
  private static final double  kAlgaeSpeedHold      = 0.1;

  private static final double  kCoralSpeedAcquire   = 0.5;
  private static final double  kCoralSpeedExpel     = -0.5;

  private static final double  kWristGearRatio      = 49.23;
  private static final double  kWristLengthMeters   = Units.inchesToMeters(15); // Simulation
  private static final double  kWristWeightKg       = Units.lbsToKilograms(20.0);  // Simulation
  private static final Voltage kWristManualVolts    = Volts.of(3.5);         // Motor voltage during manual operation (joystick)

  /** Wrist rotary motor manual move parameters */
  private enum WristMode
  {
    INIT,    // Initialize rotary
    INBOARD, // Wrist moving into the robot
    STOPPED, // Wrist stop and hold position
    OUTBOARD // Wrist moving out of the robot
  }

  private static final double       kToleranceDegrees         = 3.0;      // PID tolerance in degrees
  private static final double       kMMDebounceTime           = 0.060;    // Seconds to debounce a final position check
  private static final double       kMMMoveTimeout            = 1.0;      // Seconds allowed for a Motion Magic movement
  private static final double       kCoralDebounceTime        = 0.045;
  private static final double       kAlgaeDebounceTime        = 0.045;

  // Wrist rotary angles - Motion Magic move parameters - TODO: Update for 2025 Reefscape needs
  //    Measured hardstops and pre-defined positions:
  //               hstop  retracted   processor deployed  hstop
  //      Comp     -177.3  -176.3     -124.7    24.9      25.8
  //      Practice -177.8  -176.8     -124.7    27.3      27.4
  private static final double       kWristAngleRetracted      = Robot.isComp( ) ? -176.3 : -176.8;  // One degree from hardstops
  // private static final double       kWristAngleDeployed       = Robot.isComp( ) ? 24.9 : 27.3;    Currently being kept for reference

  private static final double       kWristAngleCoralL1        = 0.0;
  private static final double       kWristAngleCoralL2        = 0.0;
  private static final double       kWristAngleCoralL3        = 0.0;
  private static final double       kWristAngleCoralL4        = 0.0;
  private static final double       kWristAngleCoralStation   = 0.0;

  private static final double       kWristAngleAlgae23        = 0.0;
  private static final double       kWristAngleAlgae34        = 0.0;
  private static final double       kWristAngleAlgaeProcessor = 0.0;
  private static final double       kWristAngleAlgaeNet       = 0.0;

  private static final double       kWristAngleMin            = 0.0; //TODO: Complete with Correct Angles 
  private static final double       kWristAngleMax            = 0.0; // TODO: Complete with Correct Angles

  // Device objects
  private final TalonFX             m_wristMotor              = new TalonFX(Ports.kCANID_WristRotary);
  private final CANcoder            m_wristCANcoder           = new CANcoder(Ports.kCANID_WristCANcoder);
  private final TalonFX             m_clawMotor               = new TalonFX(Ports.kCANID_ClawRoller);
  private final CANrange            m_coralInClaw             = new CANrange(Ports.kCANID_CoralDetector);
  private final CANrange            m_algaeInClaw             = new CANrange(Ports.kCANID_AlgaeDetector);

  // Alerts
  private final Alert               m_rotaryAlert             =
      new Alert(String.format("%s: Wrist motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert               m_canCoderAlert           =
      new Alert(String.format("%s: CANcoder init failed!", getSubsystem( )), AlertType.kError);
  private final Alert               m_clawAlert               =
      new Alert(String.format("%s: Claw motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState     m_wristMotorSim           = m_wristMotor.getSimState( );
  private final CANcoderSimState    m_wristCANcoderSim        = m_wristCANcoder.getSimState( );
  private final CANrangeSimState    m_coralCANrangeSim        = m_coralInClaw.getSimState( );
  private final CANrangeSimState    m_algaeCANrangeSim        = m_algaeInClaw.getSimState( );
  private final SingleJointedArmSim m_armSim                  = new SingleJointedArmSim(DCMotor.getFalcon500(1), kWristGearRatio,
      SingleJointedArmSim.estimateMOI(kWristLengthMeters, kWristWeightKg), kWristLengthMeters, -Math.PI, Math.PI, false, 0.0);

  // Mechanism2d
  private final Mechanism2d         m_wristRotaryMech         = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d m_mechLigament            = m_wristRotaryMech.getRoot("Wrist", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, 0.5, 0.0, 6, new Color8Bit(Color.kPurple)));

  // Status signals
  private final StatusSignal<Angle> m_wristMotorPosition; // Default 50Hz (20ms)
  private final StatusSignal<Angle> m_ccPosition;         // Default 100Hz (10ms)

  // Declare module variables

  // Claw variables
  private boolean                   m_clawMotorValid;                 // Health indicator for motor 
  private Debouncer                 m_coralDebouncer          = new Debouncer(kCoralDebounceTime, DebounceType.kBoth);
  private Debouncer                 m_algaeDebouncer          = new Debouncer(kAlgaeDebounceTime, DebounceType.kBoth);
  private boolean                   m_coralDetected;
  private boolean                   m_algaeDetected;

  // Wrist variables
  private boolean                   m_wristMotorValid;                // Health indicator for motor 
  private boolean                   m_canCoderValid;                  // Health indicator for CANcoder 
  private double                    m_currentDegrees          = 0.0;  // Current angle in degrees
  private double                    m_targetDegrees           = 0.0;  // Target angle in degrees
  private double                    m_ccDegrees               = 0.0;  // CANcoder angle in degrees

  // Manual mode config parameters
  private VoltageOut                m_requestVolts            = new VoltageOut(Volts.of(0));
  private WristMode                 m_wristMode               = WristMode.INIT;   // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage        m_mmRequestVolts          = new MotionMagicVoltage(0).withSlot(0);
  private Debouncer                 m_mmWithinTolerance       = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                     m_mmMoveTimer             = new Timer( );     // Movement timer
  private boolean                   m_mmMoveIsFinished;           // Movement has completed (within tolerance)

  // Network tables publisher objects
  private DoublePublisher           m_clawSpeedPub;
  private DoublePublisher           m_clawSupCurPub;
  private DoublePublisher           m_wristDegreePub;

  private DoublePublisher           m_ccDegreesPub;
  private DoublePublisher           m_targetDegreesPub;
  private BooleanPublisher          m_coralDetectedPub;
  private BooleanPublisher          m_algaeDetectedPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Manipulator( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Claw motor init
    m_clawMotorValid =
        PhoenixUtil6.getInstance( ).talonFXInitialize6(m_clawMotor, kSubsystemName + "Claw", CTREConfigs6.clawRollerFXConfig( ));

    // // Initialize rotary motor and CANcoder objects
    m_wristMotorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_wristMotor, kSubsystemName + "Wrist",
        CTREConfigs6.wristRotaryFXConfig(Units.degreesToRotations(kWristAngleMin), Units.degreesToRotations(kWristAngleMax),
            Ports.kCANID_WristCANcoder, kWristGearRatio));
    m_canCoderValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_wristCANcoder, kSubsystemName + "Wrist",
        CTREConfigs6.wristRotaryCANcoderConfig( ));

    CANrangeConfiguration CANrangeConfig = new CANrangeConfiguration( );
    m_coralInClaw.getConfigurator( ).apply(CANrangeConfig);
    m_algaeInClaw.getConfigurator( ).apply(CANrangeConfig);

    m_clawAlert.set(!m_clawMotorValid);
    m_rotaryAlert.set(!m_wristMotorValid);
    m_canCoderAlert.set(!m_canCoderValid);

    // Initialize status signal objects
    m_wristMotorPosition = m_wristMotor.getPosition( );
    m_ccPosition = m_wristCANcoder.getAbsolutePosition( );

    // Initialize the elevator status signals
    Double ccRotations = (m_canCoderValid) ? m_ccPosition.refresh( ).getValue( ).in(Rotations) : 0.0;
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));
    if (m_wristMotorValid)
      m_wristMotor.setPosition(ccRotations);

    // Simulation object initialization
    m_wristMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    m_wristCANcoderSim.Orientation = ChassisReference.Clockwise_Positive;

    // Status signals
    m_wristMotorPosition.setUpdateFrequency(50);

    StatusSignal<Current> m_wristSupplyCur = m_wristMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
    StatusSignal<Current> m_wristStatorCur = m_wristMotor.getStatorCurrent( ); // Default 4Hz (250ms)
    BaseStatusSignal.setUpdateFrequencyForAll(10, m_wristSupplyCur, m_wristStatorCur);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_coralInClaw.getDistance( ), m_coralInClaw.getSignalStrength( ),
        m_coralInClaw.getIsDetected( ));
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_algaeInClaw.getDistance( ), m_algaeInClaw.getSignalStrength( ),
        m_algaeInClaw.getIsDetected( ));

    DataLogManager
        .log(String.format("%s: Update (Hz) wristPosition: %.1f wristSupplyCur: %.1f wristStatorCur: %.1f canCoderPosition: %.1f",
            getSubsystem( ), m_wristMotorPosition.getAppliedUpdateFrequency( ), m_wristSupplyCur.getAppliedUpdateFrequency( ),
            m_wristStatorCur.getAppliedUpdateFrequency( ), m_ccPosition.getAppliedUpdateFrequency( )));

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

    BaseStatusSignal.refreshAll(m_wristMotorPosition, m_ccPosition);
    m_currentDegrees = Units.rotationsToDegrees((m_wristMotorValid) ? m_wristMotorPosition.getValue( ).in(Rotations) : 0.0);
    m_ccDegrees = Units.rotationsToDegrees((m_canCoderValid) ? m_ccPosition.getValue( ).in(Rotations) : 0.0);
    m_coralDetected = m_coralDebouncer.calculate(m_coralInClaw.getIsDetected( ).getValue( ));
    m_algaeDetected = m_algaeDebouncer.calculate(m_algaeInClaw.getIsDetected( ).getValue( ));

    // // Update network table publishers
    m_clawSpeedPub.set(m_clawMotor.get( ));
    m_clawSupCurPub.set(m_clawMotor.getSupplyCurrent( ).getValueAsDouble( ));

    m_wristDegreePub.set(m_currentDegrees);
    m_ccDegreesPub.set(m_ccDegrees);
    m_targetDegreesPub.set(m_targetDegrees);
    m_coralDetectedPub.set(m_coralDetected);
    m_algaeDetectedPub.set(m_algaeDetected);
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
    m_wristMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_wristCANcoderSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_coralCANrangeSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_algaeCANrangeSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_armSim.setInputVoltage(m_wristMotorSim.getMotorVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_wristMotorSim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngleRads( ), kWristGearRatio));
    m_wristMotorSim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), kWristGearRatio));

    m_wristCANcoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads( )));
    m_wristCANcoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec( )));

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
    NetworkTable table = inst.getTable("manipulator");

    // Initialize network tables publishers
    m_clawSpeedPub = table.getDoubleTopic("clawSpeed").publish( );
    m_clawSupCurPub = table.getDoubleTopic("clawSupCur").publish( );

    m_wristDegreePub = table.getDoubleTopic("wristDegrees").publish( );
    m_ccDegreesPub = table.getDoubleTopic("ccDegrees").publish( );
    m_coralDetectedPub = table.getBooleanTopic("coralDetected").publish( );
    m_algaeDetectedPub = table.getBooleanTopic("algaeDetected").publish( );
    m_targetDegreesPub = table.getDoubleTopic("targetDegrees").publish( );

    SmartDashboard.putData("MNWristMech", m_wristRotaryMech);

    // Add commands
    SmartDashboard.putData("MNClawStop", getMoveToPositionCommand(ClawMode.STOP, this::getCurrentPosition));
    SmartDashboard.putData("MNAlgaeClawAcquire", getMoveToPositionCommand(ClawMode.ALGAEACQUIRE, this::getCurrentPosition));
    SmartDashboard.putData("MNAlgaeClawExpel", getMoveToPositionCommand(ClawMode.ALGAEEXPEL, this::getCurrentPosition));
    SmartDashboard.putData("MNAlgaeClawShoot", getMoveToPositionCommand(ClawMode.ALGAESHOOT, this::getCurrentPosition));
    SmartDashboard.putData("MNAlgaeClawHandoff", getMoveToPositionCommand(ClawMode.ALGAEPROCESSOR, this::getCurrentPosition));
    SmartDashboard.putData("MNAlgaeClawHold", getMoveToPositionCommand(ClawMode.ALGAEHOLD, this::getCurrentPosition));

    SmartDashboard.putData("MNCoralClawAcquire", getMoveToPositionCommand(ClawMode.CORALACQUIRE, this::getCurrentPosition));
    SmartDashboard.putData("MNCoralClawExpel", getMoveToPositionCommand(ClawMode.CORALEXPEL, this::getCurrentPosition));
    SmartDashboard.putData("MNCoralClawHold", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getCurrentPosition));

    SmartDashboard.putData("MNWristRetract", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorRetracted));
    SmartDashboard.putData("MNWristAngleCoral1", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorCoralL1));
    SmartDashboard.putData("MNWristAngleCoral2", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorCoralL2));
    SmartDashboard.putData("MNWristAngleCoral3", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorCoralL3));
    SmartDashboard.putData("MNWristAngleCoral4", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorCoralL4));

    SmartDashboard.putData("MNWristAngleAlgae23", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorAlgae23));
    SmartDashboard.putData("MNWristAngleAlgae34", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorAlgae34));
    SmartDashboard.putData("MNWristAngleAlgaeNet", getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorAlgaeNet));
    SmartDashboard.putData("MNWristAngleAlgaeProcessor",
        getMoveToPositionCommand(ClawMode.CORALHOLD, this::getManipulatorAlgaeProcessor));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setClawMode(ClawMode.STOP);
    setWristStopped( );

    m_targetDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! Target Degrees: %.1f", getSubsystem( ), m_targetDegrees));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_clawMotor, kSubsystemName + "Claw");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_wristMotor, kSubsystemName + "Wrist");
    PhoenixUtil6.getInstance( ).cancoderPrintFaults(m_wristCANcoder, kSubsystemName + "CANcoder");

    m_clawMotor.clearStickyFaults( );
    m_wristMotor.clearStickyFaults( );
    m_wristCANcoder.clearStickyFaults( );
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
    WristMode newMode = WristMode.INIT;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentDegrees > kWristAngleMin))
      newMode = WristMode.INBOARD;
    else if ((axisValue > 0.0) && (m_currentDegrees < kWristAngleMax))
      newMode = WristMode.OUTBOARD;
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_wristMode)
    {
      m_wristMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s %.1f deg %s", getSubsystem( ), m_wristMode, getCurrentPosition( ),
          ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_targetDegrees = m_currentDegrees;

    m_wristMotor.setControl(m_requestVolts.withOutput(kWristManualVolts.times(axisValue)));
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MOTION MAGIC /////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Initialize a Motion Magic movement and control wrist
   * 
   * @param mode
   *          claw mode to apply
   * @param newAngle
   *          rotation to move
   * @param holdPosition
   *          hold previous position if true
   */
  public void moveToPositionInit(ClawMode mode, double newAngle, boolean holdPosition)
  {
    setClawMode(mode);
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
        m_wristMotor.setControl(m_mmRequestVolts.withPosition(targetRotations));
        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getSubsystem( ),
            m_currentDegrees, m_targetDegrees, Units.degreesToRotations(m_currentDegrees), targetRotations));
      }
      else
        DataLogManager.log(String.format("%s: MM Position move target %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]",
            getSubsystem( ), m_targetDegrees, kWristAngleMin, kWristAngleMax));
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

    m_wristMotor.setControl(m_mmRequestVolts.withPosition(Units.degreesToRotations(m_targetDegrees)));

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
   * Set claw roller speed based on requested mode
   * 
   * @param mode
   *          requested mode from caller
   */
  private void setClawMode(ClawMode mode)
  {
    double output = 0.0;

    if (mode == ClawMode.ALGAEHOLD || mode == ClawMode.CORALHOLD)
    {
      DataLogManager.log(String.format("%s: Claw mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_clawMotor.get( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Claw mode is invalid: %s", getSubsystem( ), mode));
        case STOP :
          output = 0.0;
          break;
        case ALGAEACQUIRE :
          output = kAlgaeSpeedAcquire;
          break;
        case ALGAEEXPEL :
          output = kAlgaeSpeedExpel;
          break;
        case ALGAESHOOT :
          output = kAlgaeSpeedShoot;
          break;
        case ALGAEPROCESSOR :
          output = kAlgaeSpeedProcessor;
          break;
        case CORALACQUIRE :
          output = kCoralSpeedAcquire;
          break;
        case CORALEXPEL :
          output = kCoralSpeedExpel;
          break;
      }
      DataLogManager.log(String.format("%s: Claw mode is now - %s", getSubsystem( ), mode));
      m_clawMotor.set(output);
    }
  }

  /****************************************************************************
   * 
   * Set wrist rotary motor to stopped
   */
  private void setWristStopped( )
  {
    DataLogManager.log(String.format("%s: Wrist motor now STOPPED", getSubsystem( )));
    m_wristMotor.setControl(m_requestVolts.withOutput(0.0));
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
    return (degrees >= kWristAngleMin) && (degrees <= kWristAngleMax);
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
   * Return manipulator angle for retracted state
   * 
   * @return retracted state angle
   */
  public double getManipulatorRetracted( )
  {
    return kWristAngleRetracted;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 1 state
   * 
   * @return deployed state angle
   */
  public double getManipulatorCoralL1( )
  {
    return kWristAngleCoralL1;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 2 state
   * 
   * @return deployed state angle
   */
  public double getManipulatorCoralL2( )
  {
    return kWristAngleCoralL2;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 3 state
   * 
   * @return deployed state angle
   */
  public double getManipulatorCoralL3( )
  {
    return kWristAngleCoralL3;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 4 state
   * 
   * @return deployed state angle
   */
  public double getManipulatorCoralL4( )
  {
    return kWristAngleCoralL4;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral station state
   * 
   * @return deployed state angle
   */
  public double getManipulatorCoralStation( )
  {
    return kWristAngleCoralStation;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae level 23 state
   * 
   * @return deployed state angle
   */
  public double getManipulatorAlgae23( )
  {
    return kWristAngleAlgae23;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae level 34 state
   * 
   * @return deployed state angle
   */
  public double getManipulatorAlgae34( )
  {
    return kWristAngleAlgae34;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae processor state
   * 
   * @return deployed state angle
   */
  public double getManipulatorAlgaeProcessor( )
  {
    return kWristAngleAlgaeProcessor;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae net state
   * 
   * @return deployed state angle
   */
  public double getManipulatorAlgaeNet( )
  {
    return kWristAngleAlgaeNet;
  }

  /****************************************************************************
   * 
   * Return coral sensor state
   * 
   * @return true if note detected
   */
  public boolean coralDistanceInClaw( )
  {
    return m_coralDetected;
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
   *          claw mode to apply
   * @param position
   *          double supplier that provides the desired position
   * @param holdPosition
   *          boolen to indicate whether the command ever finishes
   * @return continuous command that runs wrist rotary motor
   */
  private Command getMMPositionCommand(ClawMode mode, DoubleSupplier position, boolean holdPosition)
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
   *          claw mode to apply
   * @param position
   *          double supplier that provides the desired position
   * @return continuous command that runs wrist rotary motor
   */
  public Command getMoveToPositionCommand(ClawMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, false).withName(kSubsystemName + "MMMoveToPosition");
  }

  /****************************************************************************
   * 
   * Create motion magic hold position command
   * 
   * @param mode
   *          claw mode to apply
   * @param position
   *          double supplier that provides the desired position
   * @return continuous command that runs wrist rotary motor
   */
  public Command getHoldPositionCommand(ClawMode mode, DoubleSupplier position)
  {
    return getMMPositionCommand(mode, position, true).withName(kSubsystemName + "MMHoldPosition");
  }

}
