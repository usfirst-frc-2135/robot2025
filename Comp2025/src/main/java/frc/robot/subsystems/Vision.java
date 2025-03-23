//
// Vision Subystem - handle limelight interface
//
package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;

/****************************************************************************
 * 
 * Vision subsystem class
 */
public class Vision extends SubsystemBase
{
  private static final String kLLLeftName = "limelight-left";

  /** Camera stream mode parameter */
  private enum streamMode
  {
    STANDARD(0),    //
    PIPMAIN(1),     //
    PIPSECONDARY(2);

    @SuppressWarnings("unused")
    public final int value;

    private streamMode(int value)
    {
      this.value = value;
    }
  };

  /** Camera stream mode parameter */
  private enum imuMode
  {
    EXTERNAL(0),        // Use external IMU
    EXTERNAL_SEED(1),   // Use external IMU, seed internal
    INTERNAL(2),        // Use internal
    INTERNAL_MT1(3),    // Use internal with MT1 assisted convergence
    INTERNAL_ASSIST(4)  // Use internal IMU with external IMU assisted convergence
    ;

    public final int value;

    private imuMode(int value)
    {
      this.value = value;
    }
  };

  // Constants
  private static final double kAimingKp  = 0.01;
  private static final double kDrivingKp = 0.06;

  // Objects

  // Declare module variables
  private streamMode          m_stream   = streamMode.STANDARD;

  /****************************************************************************
   * 
   * Constructor
   */
  public Vision( )
  {
    setName("Vision");
    setSubsystem("Vision");

    // Get the Network table reference once for all methods

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
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getSubsystem( )));

    LimelightHelpers.setLEDMode_ForceOff(kLLLeftName);
    LimelightHelpers.setStreamMode_PiPSecondary(kLLLeftName);

    SetIMUMode(imuMode.EXTERNAL_SEED);
  }

  /****************************************************************************
   * 
   * rotateCameraStreamMode - rotate through different stream modes
   */
  public void rotateCameraStreamMode( )
  {
    switch (m_stream)
    {
      default :
      case PIPSECONDARY :
        m_stream = streamMode.PIPMAIN;
        LimelightHelpers.setStreamMode_PiPMain(kLLLeftName);
        break;
      case PIPMAIN :
        m_stream = streamMode.STANDARD;
        LimelightHelpers.setStreamMode_Standard(kLLLeftName);
        break;
      case STANDARD :
        m_stream = streamMode.PIPSECONDARY;
        LimelightHelpers.setStreamMode_PiPSecondary(kLLLeftName);
    }

    DataLogManager.log(String.format("%s: Set stream mode (setStreamMode_PiPxxx) %s", getSubsystem( ), m_stream));
  }

  /****************************************************************************
   * 
   * Limelight auto-aiming control for rotational velocity.
   * 
   * @param maxAngularRate
   *          max angular rate to scale against
   * @return desired proportional angular velocity to rotate the chassis
   */
  public AngularVelocity aimProportional(AngularVelocity maxAngularRate)
  {
    double proportionalFactor = -LimelightHelpers.getTX(kLLLeftName) * kAimingKp;

    return maxAngularRate.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Limelight auto-ranging control for distance velocity.
   * 
   * @param maxSpeed
   *          max speed to scale against
   * @return desired proportional linear velocity in chassis forward direction
   */
  public LinearVelocity rangeProportional(LinearVelocity maxSpeed)
  {
    double proportionalFactor = LimelightHelpers.getTY(kLLLeftName) * kDrivingKp;

    return maxSpeed.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Set priorityid and display alliance color
   * 
   * @param throttle
   *          Defaults to 0. Your Limelgiht will process one frame
   *          after skipping <throttle> frames.
   */
  public void SetCPUThrottleLevel(boolean throttle)
  {
    DataLogManager.log(String.format("%s: Set Throttle level to %s", getSubsystem( ), throttle));
    // LimelightHelpers.SetThrottle("limelight", throttle ? 0 : 200);
  }

  /****************************************************************************
   * 
   * Set IMU mode as a default
   * 
   * @param mode
   *          Defaults to 0. Choose the IMU mode
   */
  public void SetIMUMode(imuMode mode)
  {
    DataLogManager.log(String.format("%s: Set IMU Mode to %s", getSubsystem( ), mode));
    LimelightHelpers.SetIMUMode(kLLLeftName, mode.value);
  }

}
