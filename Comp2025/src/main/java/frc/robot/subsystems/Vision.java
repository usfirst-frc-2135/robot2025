//
// Vision Subystem - handle limelight interface
//
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;

/****************************************************************************
 * 
 * Vision subsystem class
 */
public class Vision extends SubsystemBase
{
  private static final String kLLName = "limelight";

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

    LimelightHelpers.setLEDMode_ForceOff(kLLName);
    LimelightHelpers.setStreamMode_PiPSecondary(kLLName);

    if (DriverStation.getAlliance( ).equals(Optional.of(DriverStation.Alliance.Red)))
    {
      setPriorityId(4, "RED");
    }
    else if (DriverStation.getAlliance( ).equals(Optional.of(DriverStation.Alliance.Blue)))
    {
      setPriorityId(7, "BLUE");
    }
    else
    {
      DataLogManager.log(String.format("%s: Driver station alliance color NOT SET!", getSubsystem( )));
    }
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
        LimelightHelpers.setStreamMode_PiPMain(kLLName);
        break;
      case PIPMAIN :
        m_stream = streamMode.STANDARD;
        LimelightHelpers.setStreamMode_Standard(kLLName);
        break;
      case STANDARD :
        m_stream = streamMode.PIPSECONDARY;
        LimelightHelpers.setStreamMode_PiPSecondary(kLLName);
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
    double proportionalFactor = -LimelightHelpers.getTX(kLLName) * kAimingKp;

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
    double proportionalFactor = LimelightHelpers.getTY(kLLName) * kDrivingKp;

    return maxSpeed.times(proportionalFactor);
  }

  ///////////////////////// PRIVATE HELPERS ///////////////////////////////

  /****************************************************************************
   * 
   * Set priorityid and display alliance color
   * 
   * @param id
   *          aprilTag ID to set as priority
   * @param alliance
   *          alliance color string selected
   */
  private void setPriorityId(int id, String alliance)
  {
    DataLogManager.log(String.format("%s: Set AprilTag priority id %d (%s)", getSubsystem( ), id, alliance));
    LimelightHelpers.setPriorityTagID(kLLName, id);
  }

}
