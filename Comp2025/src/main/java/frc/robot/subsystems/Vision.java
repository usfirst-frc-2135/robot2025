//
// Vision Subystem - handle limelight interface
//
package frc.robot.subsystems;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.VIConsts;
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
  private static final double               kAimingKp  = 0.01;
  private static final double               kDrivingKp = 0.06;

  // Objects

  /* What to publish over networktables for telemetry */
  private static final NetworkTableInstance ntInst     = NetworkTableInstance.getDefault( );

  // Network tables publisher objects
  private static final NetworkTable         robotTable = ntInst.getTable(Constants.kRobotString);
  private static final IntegerSubscriber    reefLevel  = robotTable.getIntegerTopic(ELConsts.kReefLevelString).subscribe((0));
  private static final IntegerSubscriber    reefBranch = robotTable.getIntegerTopic(VIConsts.kReefBranchString).subscribe((0));

  // Declare module variables
  @SuppressWarnings("unused")
  private streamMode                        m_stream   = streamMode.STANDARD;

  /****************************************************************************
   * 
   * Constructor
   */
  public Vision( )
  {
    setName("Vision");
    setSubsystem("Vision");

    loadFieldAndDisplayPoses( );       // Identify the field and print useful poses

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

    SetCPUThrottleLevel(true);
    SetIMUModeExternalSeed( );
  }

  /****************************************************************************
   * 
   * Run vision subsystem during auto and teleop
   */
  public void run( )
  {
    DataLogManager.log(String.format("%s: Subsystem running!", getSubsystem( )));

    SetCPUThrottleLevel(false);
    SetIMUModeInternal( );
  }

  /****************************************************************************
   * 
   * Limelight auto-aiming control for rotational velocity. Only aligns the left Limelight.
   * 
   * @param maxAngularRate
   *          max angular rate to scale against
   * @param selectedBranch(left,
   *          right, center)
   * 
   * @return desired proportional angular velocity to rotate the chassis
   */
  // TODO: This should take a left/right/center parameter to decide how to select the correct camera
  // TODO: The "offset" from the 0,0 tx/ty origin should be added/subracted here
  // TODO: pseudo code of the logic:
  //
  public AngularVelocity aimProportional(int selectedBranch, AngularVelocity maxAngularRate)
  {
    double TX = 0;
    if (selectedBranch == 0)
    {
      TX = LimelightHelpers.getTX(Constants.kLLLeftName) - VIConsts.TX;
    }
    else if (selectedBranch == 2)
    {
      TX = LimelightHelpers.getTX(Constants.kLLRightName) - VIConsts.TX;
    }
    double proportionalFactor = TX * kAimingKp;
    return maxAngularRate.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Limelight auto-ranging control for distance velocity. Only aligns the left Limelight.
   * 
   * @param selectedBranch(left,
   *          right, center)
   * 
   * 
   * @param maxSpeed
   *          max speed to scale against
   * @return desired proportional linear velocity in chassis forward direction
   */
  // TODO: This should take a left/right/center parameter to decide how to select the correct camera
  // TODO: The "offset" from the 0,0 tx/ty origin should be added/subracted here
  // TODO: pseudo code of the logic:
  //
  public LinearVelocity rangeProportional(int selectedBranch, LinearVelocity maxSpeed)
  {
    double TY = 0;
    if (selectedBranch == 0)
    {
      TY = LimelightHelpers.getTY(Constants.kLLLeftName) - VIConsts.TY;
    }
    else if (selectedBranch == 2)
    {
      TY = LimelightHelpers.getTY(Constants.kLLRightName) - VIConsts.TY;
    }
    double proportionalFactor = TY * kDrivingKp;
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
    LimelightHelpers.SetThrottle(Constants.kLLLeftName, throttle ? 0 : 100);
    LimelightHelpers.SetThrottle(Constants.kLLRightName, throttle ? 0 : 100);
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

  ////////////////////////////////////////////////////////////////////////////
  /////////////////////// AUTO-ALIGN HELPERS /////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Print out field layout, tag ID poses, and scoring poses
   */
  private void loadFieldAndDisplayPoses( )
  {
    // Identify the field and load it (any reference loads it)
    DataLogManager.log(String.format("Field: %s width %.2f length %.2f", VIConsts.kGameField, VIConsts.kATField.getFieldWidth( ),
        VIConsts.kATField.getFieldLength( )));

    // DataLogManager.log(String.format("-----"));

    // for (int i = 1; i <= 22; i++)
    // {
    //   DataLogManager.log(String.format("Field: ID %2d %s", i, VIConsts.kATField.getTagPose(i)));
    // }

    // DataLogManager.log(String.format("-----"));

    for (int tag = 17; tag <= 22; tag++)
    {
      getScoringGoalPose(tag, VIConsts.ReefBranch.LEFT.value);
      getScoringGoalPose(tag, VIConsts.ReefBranch.ALGAE.value);
      getScoringGoalPose(tag, VIConsts.ReefBranch.RIGHT.value);
    }

    // DataLogManager.log(String.format("-----"));

    // for (int tag = 6; tag <= 11; tag++)
    // {
    //   getScoringGoalPose(tag, VIConsts.ReefBranch.LEFT.value);
    //   getScoringGoalPose(tag, VIConsts.ReefBranch.ALGAE.value);
    //   getScoringGoalPose(tag, VIConsts.ReefBranch.RIGHT.value);
    // }

    // DataLogManager.log(String.format("-----"));
  }

  /****************************************************************************
   *
   * Initialize the arrays with the reef AprilTags
   */
  private static final int[ ] blueReefTags =
  {
      17, 18, 19, 20, 21, 22  // Must be in numerical order
  };

  private static final int[ ] redReefTags  =
  {
      8, 7, 6, 11, 10, 9      // Rotationally ordered the same as blue tags above
  };

  /****************************************************************************
   *
   * Find the closest AprilTag ID to the robot and return it (always returns blue side tags only)
   * 
   * @return closestBlueTag
   *         AprilTag closest to current pose
   */
  private static int findClosestReefTag(Pose2d currentPose)
  {
    if (DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red)
    {
      currentPose = FlippingUtil.flipFieldPose(currentPose);
    }

    int closestBlueTag = 0;                                                 // Variable for saving the tag with the shortest distance (0 means none found)
    double shortestDistance = Units.feetToMeters(57.0);                // field length in meters - Variable for keeping track of lowest distance (57.0 means none found)

    // Just one calculation for either tag set
    for (int i = 0; i < 6; i++)                                             // Iterate through the array of selected reef tags
    {
      Pose2d atPose = VIConsts.kATField.getTagPose(blueReefTags[i]).get( ).toPose2d( );         // Get the AT tag in Pose2d form
      double distance = currentPose.getTranslation( ).getDistance((atPose.getTranslation( )));  // Calculate the distance from the AT tag to the robotPose
      DataLogManager.log(String.format("Vision: Possible tag: %d pose: %s distance: %f", blueReefTags[i], atPose, distance));
      if (distance < shortestDistance)                                                          // If the distance is shorter than what was saved before
      {
        closestBlueTag = blueReefTags[i];                                                       // Saves cloest AT id (always in blue space)
        shortestDistance = distance;                                                            // Update new shortest distance
      }
    }

    DataLogManager.log(String.format("Vision: closest tag: %d current pose: %s - FOUND!", closestBlueTag, currentPose));
    return closestBlueTag;
  }

  /****************************************************************************
   * 
   * Calculate a scoring waypoint for a given tag ID and branch (left, center, right)
   */
  private static Pose2d getScoringGoalPose(int tag, int branch)
  {
    Pose2d atPose = VIConsts.kATField.getTagPose(tag).orElse(new Pose3d( )).toPose2d( );
    Transform2d branchOffset;

    switch (branch)
    {
      case 0 :  // Left
        branchOffset = Constants.kBranchScoreLeft;
        break;
      default :
      case 1 :  // Algae
        branchOffset = Constants.kBranchScoreCenter;
        break;
      case 2 :  // Right
        branchOffset = Constants.kBranchScoreRight;
        break;
    }

    Pose2d pose = atPose.transformBy(branchOffset);
    DataLogManager.log(String.format("AT %2d Pose %-79s waypoint %-79s", tag, atPose, pose));
    return pose;
  }

  /****************************************************************************
   * 
   * * Find Goal Pose for a given blue reef AprilTag ID
   * 
   * 1) Get the closest reef AprilTag
   * 2) Retrive the a branch/face offset selection (left, middle (algae), right)
   * 3) Use the closest blue AprilTag ID and branch offset to find goal pose
   * 4) Return the goal pose (blue side only)
   * 
   * @return goalPose
   *         goal pose for the reef tag passed in
   */
  public static Pose2d findGoalPose(Pose2d currentPose)
  {
    int reefTag = findClosestReefTag(currentPose);

    int reefOffset = (int) reefBranch.get( );

    int relativeReefTag = reefTag - blueReefTags[0];
    Pose2d goalPose = getScoringGoalPose(reefTag, reefOffset);

    if (DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red)
    {
      goalPose = FlippingUtil.flipFieldPose(goalPose);
      reefTag = redReefTags[relativeReefTag];
    }

    DataLogManager.log(String.format("Vision: branch: %d goal tag: %d goal pose %s", reefLevel.get( ), reefTag, goalPose));
    return goalPose;
  }

}
