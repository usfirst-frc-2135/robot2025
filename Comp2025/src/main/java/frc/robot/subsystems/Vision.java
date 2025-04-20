//
// Vision Subystem - handle limelight interface
//
package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.LimelightHelpers;

/****************************************************************************
 * 
 * Vision subsystem class
 */
public class Vision extends SubsystemBase
{

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
  @SuppressWarnings("unused")
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

    LimelightHelpers.setLEDMode_ForceOff(Constants.kLLLeftName);          // These work on LL3 and lower (not LL4)
    LimelightHelpers.setLEDMode_ForceOff(Constants.kLLRightName);         // These work on LL3 and lower (not LL4)
    LimelightHelpers.setStreamMode_PiPSecondary(Constants.kLLLeftName);   // These work on LL3 and lower (not LL4)
    LimelightHelpers.setStreamMode_PiPSecondary(Constants.kLLRightName);  // These work on LL3 and lower (not LL4)

    SetIMUModeExternalSeed( );
  }

  /****************************************************************************
   * 
   * Limelight auto-aiming control for rotational velocity. Only aligns the left Limelight.
   * 
   * @param maxAngularRate
   *          max angular rate to scale against
   * @return desired proportional angular velocity to rotate the chassis
   */
  public AngularVelocity aimProportional(AngularVelocity maxAngularRate)
  {
    double proportionalFactor = -LimelightHelpers.getTX(Constants.kLLLeftName) * kAimingKp;

    return maxAngularRate.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Limelight auto-ranging control for distance velocity. Only aligns the left Limelight.
   * 
   * @param maxSpeed
   *          max speed to scale against
   * @return desired proportional linear velocity in chassis forward direction
   */
  public LinearVelocity rangeProportional(LinearVelocity maxSpeed)
  {
    double proportionalFactor = LimelightHelpers.getTY(Constants.kLLLeftName) * kDrivingKp;

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
    // LimelightHelpers.SetThrottle(kLLLeftName, throttle ? 0 : 200);
    // LimelightHelpers.SetThrottle(kLLRightName, throttle ? 0 : 200);
  }

  /****************************************************************************
   * 
   * Set IMU mode to EXTERNAL_SEED mode (load the LL4 internal IMU from robot IMU)
   * 
   */
  public void SetIMUModeExternalSeed( )
  {
    final imuMode mode = imuMode.EXTERNAL_SEED;
    DataLogManager.log(String.format("%s: Set IMU Mode to %d (%s)", getSubsystem( ), mode.value, mode));
    // LimelightHelpers.SetIMUMode(Constants.kLLLeftName, mode.value);
    // LimelightHelpers.SetIMUMode(Constants.kLLRightName, mode.value);
  }

  /****************************************************************************
   * 
   * Set IMU mode to INTERNAL mode (use the LL4 internal IMU)
   * 
   */
  public void SetIMUModeInternal( )
  {
    final imuMode mode = imuMode.INTERNAL;
    DataLogManager.log(String.format("%s: Set IMU Mode to %d (%s)", getSubsystem( ), mode.value, mode));
    // LimelightHelpers.SetIMUMode(Constants.kLLLeftName, mode.value);
    // LimelightHelpers.SetIMUMode(Constants.kLLRightName, mode.value);
  }

}
