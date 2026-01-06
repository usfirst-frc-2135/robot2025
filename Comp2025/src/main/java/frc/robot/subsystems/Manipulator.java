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
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import edu.wpi.first.networktables.IntegerSubscriber;
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
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.Ports;
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
  private static final String       kSubsystemName            = "Manipulator";
  private static final double       kWristGearRatio           = 49.23;
  private static final double       kWristLengthMeters        = Units.inchesToMeters(13); // Simulation
  private static final double       kWristWeightKg            = Units.lbsToKilograms(8.0);  // Simulation
  private static final Voltage      kWristManualVolts         = Volts.of(3.5);         // Motor voltage during manual operation (joystick)

  private static final double       kToleranceDegrees         = 3.0;      // PID tolerance in degrees
  private static final double       kMMDebounceTime           = 0.040;    // Seconds to debounce a final angle check
  private static final double       kMMMoveTimeout            = 1.0;      // Seconds allowed for a Motion Magic movement

  // Wrist rotary angles - Motion Magic move parameters
  //    Measured hardstops and pre-defined positions:
  //               hstop    hstop
  //      Comp     -119.0   52.0
  //      Practice -119.0   52.0
  private static final double       kWristAngleMin            = -130.4;
  private static final double       kWristAngleMax            = 51.0;

  private static final double       kWristAngleSafeState      = -103.0;

  private static final double       kWristAngleCoralStation   = -130.4;
  private static final double       kWristAngleCoralL1        = -41;
  private static final double       kWristAngleCoralL2        = -103.0;
  private static final double       kWristAngleCoralL3        = -103.0;
  private static final double       kWristAngleCoralL4        = -84.5;

  private static final double       kWristAngleAlgae23        = 29.0;
  private static final double       kWristAngleAlgae34        = 29.0;
  private static final double       kWristAngleAlgaeProcessor = 50.0;
  private static final double       kWristAngleAlgaeNet       = -10.0;

  // Claw roller speeds
  private static final DutyCycleOut kClawRollerStop           = new DutyCycleOut(0.0).withIgnoreHardwareLimits(true);

  private static final DutyCycleOut kCoralSpeedAcquire        = new DutyCycleOut(-0.6).withIgnoreHardwareLimits(false);
  private static final DutyCycleOut kCoralSpeedExpel          = new DutyCycleOut(-0.42).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kCoralSpeedExpelL1        = new DutyCycleOut(-0.32).withIgnoreHardwareLimits(true);

  private static final DutyCycleOut kAlgaeSpeedAcquire        = new DutyCycleOut(0.5).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kAlgaeSpeedExpel          = new DutyCycleOut(-0.27).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kAlgaeSpeedShoot          = new DutyCycleOut(-1.0).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kAlgaeSpeedProcessor      = new DutyCycleOut(-0.27).withIgnoreHardwareLimits(true);
  private static final DutyCycleOut kAlgaeSpeedHold           = new DutyCycleOut(0.2).withIgnoreHardwareLimits(true);

  /** Wrist rotary motor manual move parameters */
  public enum JoystickMode
  {
    INIT,     // Initialize rotary
    INBOARD,  // Wrist moving into the robot
    STOP,     // Wrist stop and hold angle
    OUTBOARD  // Wrist moving out of the robot
  }

  // Device objects
  private final TalonFX               m_wristMotor        = new TalonFX(Ports.kCANID_WristRotary);
  private final CANcoder              m_wristCANcoder     = new CANcoder(Ports.kCANID_WristCANcoder);
  private final TalonFX               m_clawMotor         = new TalonFX(Ports.kCANID_ClawRoller);
  private final CANrange              m_coralDetector     = new CANrange(Ports.kCANID_CoralDetector);
  private final CANrange              m_algaeDetector     = new CANrange(Ports.kCANID_AlgaeDetector);

  // Alerts
  private final Alert                 m_rotaryAlert       =
      new Alert(String.format("%s: Wrist rotary motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                 m_canCoderAlert     =
      new Alert(String.format("%s: Wrist CANcoder init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                 m_clawAlert         =
      new Alert(String.format("%s: Claw roller motor init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                 m_coralCRAlert      =
      new Alert(String.format("%s: Coral detector init failed!", getSubsystem( )), AlertType.kError);
  private final Alert                 m_algaeCRAlert      =
      new Alert(String.format("%s: Algae detector init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState       m_wristMotorSim     = m_wristMotor.getSimState( );
  private final CANcoderSimState      m_wristCANcoderSim  = m_wristCANcoder.getSimState( );
  private final SingleJointedArmSim   m_armSim            = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), kWristGearRatio,
      SingleJointedArmSim.estimateMOI(kWristLengthMeters, kWristWeightKg), kWristLengthMeters, -Math.PI, Math.PI, false, 0.0);
  private final TalonFXSimState       m_clawMotorSim      = m_clawMotor.getSimState( );
  private final CANrangeSimState      m_coralDetectedSim  = m_coralDetector.getSimState( );

  // Mechanism2d
  private final Mechanism2d           m_wristRotaryMech   = new Mechanism2d(1.0, 1.0);
  private final MechanismLigament2d   m_mechLigament      = m_wristRotaryMech.getRoot("Wrist", 0.5, 0.5)
      .append(new MechanismLigament2d(kSubsystemName, 0.5, 0.0, 6, new Color8Bit(Color.kPurple)));

  // Status signals for sensors
  private final StatusSignal<Angle>   m_wristMotorPosition; // Default 50Hz (20ms)
  private final StatusSignal<Angle>   m_ccPosition;         // Default 100Hz (10ms)
  private final StatusSignal<Boolean> m_coralIsDetected;    // Default 50Hz (20ms)
  private final StatusSignal<Boolean> m_algaeIsDetected;    // Default 50Hz (20ms)

  // Declare module variables
  // Wrist rotary
  private boolean                     m_wristMotorValid;                // Health indicator for motor 
  private boolean                     m_canCoderValid;                  // Health indicator for CANcoder 
  private boolean                     m_clawMotorValid;                 // Health indicator for motor 
  private double                      m_currentDegrees    = 0.0;  // Current angle in degrees
  private double                      m_goalDegrees       = 0.0;  // Goal angle in degrees
  private double                      m_ccDegrees         = 0.0;  // CANcoder angle in degrees

  // Coral detector
  private boolean                     m_coralDetectorValid;             // Health indicator for CANrange
  private boolean                     m_coralDetected;

  // Algae detector
  private boolean                     m_algaeDetectorValid;             // Health indicator for CANrange
  private boolean                     m_algaeDetected;

  // Claw Roller
  private DutyCycleOut                m_clawRequestVolts  = kClawRollerStop;

  // Manual mode config parameters
  private VoltageOut                  m_wristRequestVolts = new VoltageOut(Volts.of(0)).withEnableFOC(true);
  private JoystickMode                m_manualMode        = JoystickMode.INIT;   // Manual movement mode with joysticks

  // Motion Magic config parameters
  private MotionMagicVoltage          m_mmRequestVolts    = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private Debouncer                   m_mmWithinTolerance = new Debouncer(kMMDebounceTime, DebounceType.kRising);
  private Timer                       m_mmMoveTimer       = new Timer( );     // Movement timer
  private boolean                     m_mmMoveIsFinished  = true;             // Movement has completed (within tolerance)

  // Network tables publisher objects
  private DoublePublisher             m_clawSpeedPub;
  private DoublePublisher             m_wristDegreePub;
  private DoublePublisher             m_ccDegreesPub;
  private DoublePublisher             m_goalDegreesPub;
  private BooleanPublisher            m_coralDetectedPub;
  private BooleanPublisher            m_algaeDetectedPub;

  // Network tables subscriber objects
  private final NetworkTableInstance  kNTInst             = NetworkTableInstance.getDefault( );
  private final NetworkTable          kRobotTable         = kNTInst.getTable(Constants.kRobotString);
  private IntegerSubscriber           m_reefLevel         = kRobotTable.getIntegerTopic(ELConsts.kReefLevelString).subscribe((0));

  /****************************************************************************
   * 
   * Constructor
   */
  public Manipulator( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Claw motor init
    m_clawMotorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_clawMotor, kSubsystemName + "Claw",
        CTREConfigs6.clawRollerFXConfig(m_coralDetector.getDeviceID( )));

    // // Initialize rotary motor and CANcoder objects
    m_wristMotorValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_wristMotor, kSubsystemName + "Wrist",
        CTREConfigs6.wristRotaryFXConfig(Units.degreesToRotations(kWristAngleMin), Units.degreesToRotations(kWristAngleMax),
            Ports.kCANID_WristCANcoder, kWristGearRatio));
    m_canCoderValid = PhoenixUtil6.getInstance( ).canCoderInitialize6(m_wristCANcoder, kSubsystemName + "Wrist",
        CTREConfigs6.wristRotaryCANcoderConfig( ));

    m_coralDetectorValid = m_coralDetector.getConfigurator( ).apply(CTREConfigs6.coralCANRangeConfig( )).isOK( );
    m_algaeDetectorValid = m_algaeDetector.getConfigurator( ).apply(CTREConfigs6.algaeCANRangeConfig( )).isOK( );

    m_clawAlert.set(!m_clawMotorValid);
    m_rotaryAlert.set(!m_wristMotorValid);
    m_canCoderAlert.set(!m_canCoderValid);
    m_coralCRAlert.set(!m_coralDetectorValid);
    m_algaeCRAlert.set(!m_algaeDetectorValid);

    // Initialize status signal objects
    m_wristMotorPosition = m_wristMotor.getPosition( );
    m_ccPosition = m_wristCANcoder.getAbsolutePosition( );
    m_coralIsDetected = m_coralDetector.getIsDetected( );
    m_algaeIsDetected = m_algaeDetector.getIsDetected( );

    // Initialize the motor status signals
    Double ccRotations = (m_canCoderValid) ? m_ccPosition.refresh( ).getValue( ).in(Rotations) : 0.0;
    m_currentDegrees = Units.rotationsToDegrees(ccRotations);
    DataLogManager.log(String.format("%s: CANcoder initial degrees %.1f", getSubsystem( ), m_currentDegrees));

    if (m_wristMotorValid)
    {
      m_wristMotor.setPosition(ccRotations);

      // Status signals
      m_wristMotorPosition.setUpdateFrequency(50);
      StatusSignal<Current> m_wristSupplyCur = m_wristMotor.getSupplyCurrent( ); // Default 4Hz (250ms)
      StatusSignal<Current> m_wristStatorCur = m_wristMotor.getStatorCurrent( ); // Default 4Hz (250ms)
      BaseStatusSignal.setUpdateFrequencyForAll(10, m_wristSupplyCur, m_wristStatorCur);

      DataLogManager.log(
          String.format("%s: Update (Hz) wristPosition: %.1f wristSupplyCur: %.1f wristStatorCur: %.1f canCoderPosition: %.1f",
              getSubsystem( ), m_wristMotorPosition.getAppliedUpdateFrequency( ), m_wristSupplyCur.getAppliedUpdateFrequency( ),
              m_wristStatorCur.getAppliedUpdateFrequency( ), m_ccPosition.getAppliedUpdateFrequency( )));
    }

    if (m_coralDetectorValid && m_algaeDetectorValid)
    {
      BaseStatusSignal.setUpdateFrequencyForAll(100, m_coralIsDetected, m_algaeIsDetected);

      DataLogManager.log(String.format("%s: Update (Hz) coralIsDetected: %s algaeIsDetected: %s", getSubsystem( ),
          m_coralIsDetected.getAppliedUpdateFrequency( ), m_algaeIsDetected.getAppliedUpdateFrequency( )));
    }

    DataLogManager.log(String.format("%s: Initial position %.1f degrees", getSubsystem( ), m_currentDegrees));

    // Simulation object initialization
    m_wristMotorSim.Orientation = ChassisReference.Clockwise_Positive;
    m_wristCANcoderSim.Orientation = ChassisReference.CounterClockwise_Positive;

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
    m_coralDetected = m_coralDetector.getIsDetected( ).getValue( );
    m_algaeDetected = m_algaeDetector.getIsDetected( ).getValue( );

    // // Update network table publishers
    m_clawSpeedPub.set(m_clawMotor.get( ));

    m_wristDegreePub.set(m_currentDegrees);
    m_ccDegreesPub.set(m_ccDegrees);
    m_goalDegreesPub.set(m_goalDegrees);
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
    m_armSim.setInputVoltage(m_wristMotorSim.getMotorVoltage( ));
    m_clawMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));
    m_coralDetectedSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    // update for 20 msec loop
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_wristMotorSim.setRawRotorPosition(Conversions.radiansToInputRotations(m_armSim.getAngleRads( ), kWristGearRatio));
    m_wristMotorSim.setRotorVelocity(Conversions.radiansToInputRotations(m_armSim.getVelocityRadPerSec( ), kWristGearRatio));

    m_wristCANcoderSim.setRawPosition(Units.radiansToRotations(m_armSim.getAngleRads( )));
    m_wristCANcoderSim.setVelocity(Units.radiansToRotations(m_armSim.getVelocityRadPerSec( )));

    m_clawMotorSim.setRawRotorPosition((5300 / 60 / 50) * m_clawMotor.get( ));
    m_clawMotorSim.setRotorVelocity((5300 / 60) * m_clawMotor.get( ));

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
    NetworkTable table = NetworkTableInstance.getDefault( ).getTable("manipulator");

    // Initialize network tables publishers
    m_clawSpeedPub = table.getDoubleTopic("clawSpeed").publish( );

    m_wristDegreePub = table.getDoubleTopic("wristDegrees").publish( );
    m_ccDegreesPub = table.getDoubleTopic("ccDegrees").publish( );
    m_coralDetectedPub = table.getBooleanTopic("coralDetected").publish( );
    m_algaeDetectedPub = table.getBooleanTopic("algaeDetected").publish( );
    m_goalDegreesPub = table.getDoubleTopic("goalDegrees").publish( );

    SmartDashboard.putData("MNWristMech", m_wristRotaryMech);

    // Add commands
    SmartDashboard.putData("MNClawStop", getMoveToPositionCommand(ClawMode.STOP, this::getCurrentAngle));

    SmartDashboard.putData("MNAlgaeAcquire", getMoveToPositionCommand(ClawMode.ALGAEACQUIRE, this::getCurrentAngle));
    SmartDashboard.putData("MNAlgaeHold", getMoveToPositionCommand(ClawMode.ALGAEHOLD, this::getCurrentAngle));
    SmartDashboard.putData("MNAlgaeExpel", getMoveToPositionCommand(ClawMode.ALGAEEXPEL, this::getCurrentAngle));
    SmartDashboard.putData("MNAlgaeShoot", getMoveToPositionCommand(ClawMode.ALGAESHOOT, this::getCurrentAngle));
    SmartDashboard.putData("MNAlgaeProcessor", getMoveToPositionCommand(ClawMode.ALGAEPROCESSOR, this::getCurrentAngle));
    SmartDashboard.putData("MNAlgaeMaintain", getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, this::getCurrentAngle));

    SmartDashboard.putData("MNCoralAcquire", getMoveToPositionCommand(ClawMode.CORALACQUIRE, this::getCurrentAngle));
    SmartDashboard.putData("MNCoralExpel", getMoveToPositionCommand(ClawMode.CORALEXPEL, this::getCurrentAngle));
    SmartDashboard.putData("MNCoralHold", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getCurrentAngle));

    SmartDashboard.putData("MNWristSafeState", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleSafeState));

    SmartDashboard.putData("MNWristCoralStation", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleCoralStation));
    SmartDashboard.putData("MNWristCoralL1", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleCoralL1));
    SmartDashboard.putData("MNWristCoralL2", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleCoralL2));
    SmartDashboard.putData("MNWristCoralL3", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleCoralL3));
    SmartDashboard.putData("MNWristCoralL4", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleCoralL4));

    SmartDashboard.putData("MNWristAlgaeL23", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleAlgae23));
    SmartDashboard.putData("MNWristAlgaeL34", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleAlgae34));
    SmartDashboard.putData("MNWristAlgaeProcessor",
        getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleAlgaeProcessor));
    SmartDashboard.putData("MNWristAlgaeNet", getMoveToPositionCommand(ClawMode.CORALMAINTAIN, this::getAngleAlgaeNet));
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

    m_goalDegrees = m_currentDegrees;
    DataLogManager.log(String.format("%s: Subsystem initialized! goal Degrees: %.1f", getSubsystem( ), m_goalDegrees));
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_clawMotor, kSubsystemName + "Claw");
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_wristMotor, kSubsystemName + "Wrist");
    PhoenixUtil6.getInstance( ).canCoderPrintFaults(m_wristCANcoder, kSubsystemName + "CANcoder");
    PhoenixUtil6.getInstance( ).canRangePrintFaults(m_coralDetector, kSubsystemName + "CoralDetector");
    PhoenixUtil6.getInstance( ).canRangePrintFaults(m_algaeDetector, kSubsystemName + "AlgaeDetector");

    m_clawMotor.clearStickyFaults( );
    m_wristMotor.clearStickyFaults( );
    m_wristCANcoder.clearStickyFaults( );
    m_coralDetector.clearStickyFaults( );
    m_algaeDetector.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// MANUAL MOVEMENT //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  private final static Voltage kManualKG = Volts.of(-0.25);

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
    JoystickMode newMode = JoystickMode.STOP;

    axisValue = MathUtil.applyDeadband(axisValue, Constants.kStickDeadband);

    if ((axisValue < 0.0) && (m_currentDegrees > kWristAngleMin))
    {
      newMode = JoystickMode.INBOARD;
    }
    else if ((axisValue > 0.0) && (m_currentDegrees < kWristAngleMax))
    {
      newMode = JoystickMode.OUTBOARD;
    }
    else
    {
      rangeLimited = true;
      axisValue = 0.0;
    }

    if (newMode != m_manualMode)
    {
      m_manualMode = newMode;
      DataLogManager.log(String.format("%s: Manual move mode %s now %.1f deg %s", getSubsystem( ), m_manualMode,
          getCurrentAngle( ), ((rangeLimited) ? " - RANGE LIMITED" : "")));
    }

    m_goalDegrees = m_currentDegrees;

    m_wristMotor.setControl(m_wristRequestVolts.withOutput(
        kWristManualVolts.times(axisValue).plus(kManualKG.times(Math.cos(Units.degreesToRadians(m_currentDegrees))))));
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
  private void moveToPositionInit(ClawMode mode, double newAngle, boolean holdPosition)
  {
    setClawMode(mode);
    m_mmMoveTimer.restart( );

    if (holdPosition)
    {
      newAngle = getCurrentAngle( );
    }

    // Decide if a new position request
    if (holdPosition || newAngle != m_goalDegrees || !MathUtil.isNear(newAngle, m_currentDegrees, kToleranceDegrees))
    {
      // Validate the position request
      if (isMoveValid(newAngle))
      {
        m_goalDegrees = newAngle;
        m_mmMoveIsFinished = false;
        m_mmWithinTolerance.calculate(false); // Reset the debounce filter

        double goalRotations = Units.degreesToRotations(m_goalDegrees);
        m_wristMotor.setControl(m_mmRequestVolts.withPosition(goalRotations));
        DataLogManager.log(String.format("%s: MM Position move: %.1f -> %.1f degrees (%.3f -> %.3f rot)", getSubsystem( ),
            m_currentDegrees, m_goalDegrees, Units.degreesToRotations(m_currentDegrees), goalRotations));
      }
      else
      {
        DataLogManager.log(String.format("%s: MM Position move goal %.1f degrees is OUT OF RANGE! [%.1f, %.1f deg]",
            getSubsystem( ), m_goalDegrees, kWristAngleMin, kWristAngleMax));
      }
    }
    else
    {
      m_mmMoveIsFinished = true;
      DataLogManager.log(String.format("%s: MM Position already achieved - goal %s degrees", getSubsystem( ), m_goalDegrees));
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
    double error = m_goalDegrees - m_currentDegrees;

    m_wristMotor.setControl(m_mmRequestVolts.withPosition(Units.degreesToRotations(m_goalDegrees)));

    if (holdPosition)
    {
      return false;
    }

    if (m_mmWithinTolerance.calculate(Math.abs(error) < kToleranceDegrees) || timedOut)
    {
      if (!m_mmMoveIsFinished)
      {
        DataLogManager
            .log(String.format("%s: MM Position move finished - Current degrees: %.1f (difference %.1f) - Time: %.3f sec %s",
                getSubsystem( ), m_currentDegrees, error, m_mmMoveTimer.get( ), (timedOut) ? "- Warning: TIMED OUT!" : ""));
      }
      SmartDashboard.putNumber("MNMoveTime", m_mmMoveTimer.get( ) - kMMDebounceTime);

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
    m_clawRequestVolts = kClawRollerStop;

    if (mode == ClawMode.ALGAEMAINTAIN || mode == ClawMode.CORALMAINTAIN)
    {
      DataLogManager.log(String.format("%s: Claw mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_clawMotor.get( )));
    }
    else
    {
      // if (mode == ClawMode.ALGAEHOLD)
      // {
      //   double rotations = m_clawMotor.getPosition( ).getValueAsDouble( );
      //   Slot0Configs slot0Configs = new Slot0Configs( ).withKP(25);
      //   m_clawMotor.getConfigurator( ).apply(slot0Configs);
      //   PositionDutyCycle positionDutyCycle = new PositionDutyCycle(rotations).withSlot(0).withEnableFOC(true);
      //   m_clawMotor.setControl(positionDutyCycle);
      // }
      // else
      {
        switch (mode)
        {
          default :
            DataLogManager.log(String.format("%s: Claw mode is invalid: %s", getSubsystem( ), mode));
          case STOP :
            m_clawRequestVolts = (m_algaeDetected) ? kAlgaeSpeedHold : kClawRollerStop;
            break;
          case ALGAEACQUIRE :
            m_clawRequestVolts = kAlgaeSpeedAcquire;
            break;
          case ALGAEEXPEL :
            m_clawRequestVolts = kAlgaeSpeedExpel;
            break;
          case ALGAESHOOT :
            m_clawRequestVolts = kAlgaeSpeedShoot;
            break;
          case ALGAEPROCESSOR :
            m_clawRequestVolts = kAlgaeSpeedProcessor;
            break;
          case CORALACQUIRE :
            m_clawRequestVolts = kCoralSpeedAcquire;
            break;
          case CORALEXPEL :
            m_clawRequestVolts = ((int) m_reefLevel.get( ) == 1) ? kCoralSpeedExpelL1 : kCoralSpeedExpel;
            DataLogManager.log(String.format("%s: reefLevel.get is %d", getSubsystem( ), (int) m_reefLevel.get( )));
            break;
          case ALGAEHOLD :  // Special case above the switch - this case doesn't execute!
            m_clawRequestVolts = kAlgaeSpeedHold;
            break;
        }

        m_clawMotor.setControl(m_clawRequestVolts);
      }

      DataLogManager.log(String.format("%s: Claw mode is now - %s", getSubsystem( ), mode));
    }
  }

  /****************************************************************************
   * 
   * Set wrist rotary motor to stopped
   */
  private void setWristStopped( )
  {
    DataLogManager.log(String.format("%s: Wrist motor now STOPPED", getSubsystem( )));
    m_wristMotor.setControl(m_wristRequestVolts.withOutput(0.0));
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
   * Return current angle
   * 
   * @return current rotary angle
   */
  public double getCurrentAngle( )
  {
    return m_currentDegrees;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for Safe Position
   * 
   * @return safe state angle
   */
  public double getAngleSafeState( )
  {
    return kWristAngleSafeState;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 1 state
   * 
   * @return coral L1 angle
   */
  public double getAngleCoralL1( )
  {
    return kWristAngleCoralL1;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 2 state
   * 
   * @return coral L2 angle
   */
  public double getAngleCoralL2( )
  {
    return kWristAngleCoralL2;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 3 state
   * 
   * @return coral L3 angle
   */
  public double getAngleCoralL3( )
  {
    return kWristAngleCoralL3;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral level 4 state
   * 
   * @return coral L4 angle
   */
  public double getAngleCoralL4( )
  {
    return kWristAngleCoralL4;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for coral station state
   * 
   * @return coral station angle
   */
  public double getAngleCoralStation( )
  {
    return kWristAngleCoralStation;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae level 23 state
   * 
   * @return algae L23 angle
   */
  public double getAngleAlgae23( )
  {
    return kWristAngleAlgae23;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae level 34 state
   * 
   * @return algae L34 angle
   */
  public double getAngleAlgae34( )
  {
    return kWristAngleAlgae34;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae processor state
   * 
   * @return algae processor angle
   */
  public double getAngleAlgaeProcessor( )
  {
    return kWristAngleAlgaeProcessor;
  }

  /****************************************************************************
   * 
   * Return manipulator angle for algae net state
   * 
   * @return algae net angle
   */
  public double getAngleAlgaeNet( )
  {
    return kWristAngleAlgaeNet;
  }

  /****************************************************************************
   * 
   * Return coral sensor state
   * 
   * @return true if coral detected
   */
  public boolean isCoralDetected( )
  {
    return m_coralDetected;
  }

  /****************************************************************************
   * 
   * Return coral sensor state
   * 
   * @return true if coral not present
   */
  public boolean isCoralExpelled( )
  {
    return !m_coralDetected;
  }

  /****************************************************************************
   * 
   * Return algae sensor state
   * 
   * @return true if algae detected
   */
  public boolean isAlgaeDetected( )
  {
    return m_algaeDetected;
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
