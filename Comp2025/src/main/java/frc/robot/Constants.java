
package frc.robot;

import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants
{
  // bot serial nums
  public static final String  kCompSN               = "032B1F7E"; // TODO: update for 2025 roboRIO serial numbers
  public static final String  kBetaSN               = "03260A3A";

  // Game controller definitions
  public static final int     kDriverPadPort        = 0;
  public static final int     kOperatorPadPort      = 1;

  public static final double  kStickDeadband        = 0.15;
  public static final double  kTriggerThreshold     = 0.25;

  public static final boolean kRumbleOn             = true;
  public static final boolean kRumbleOff            = false;
  public static final double  kRumbleIntensity      = 0.5;  // 0.0 is low, 1.0 is high

  // Phoenix firmware versions expected
  public static final int     kPhoenix5MajorVersion = ((22 * 256) + 0); // TODO: update CTRE version numbers for 2025
  public static final int     kPhoenix6MajorVersion = 24;

  /****************************************************************************
   * CAN IDs and PWM IDs
   ****************************************************************************/
  public static final class Ports
  {
    public static final String kCANCarnivore         = "canivore1";
    public static final String kCANRio               = "rio";

    // CANivore CAN IDs - Swerve
    public static final int    kCANID_DriveLF        = 1;    // Kraken X60
    public static final int    kCANID_SteerLF        = 2;    // Kraken X60
    public static final int    kCANID_CANcoderLF     = 3;    // CANcoder

    public static final int    kCANID_DriveRF        = 4;    // Kraken X60     
    public static final int    kCANID_SteerRF        = 5;    // Kraken X60
    public static final int    kCANID_CANcoderRF     = 6;    // CANcoder

    public static final int    kCANID_DriveLR        = 7;    // Kraken X60
    public static final int    kCANID_SteerLR        = 8;    // Kraken X60
    public static final int    kCANID_CANcoderLR     = 9;    // CANcoder

    public static final int    kCANID_DriveRR        = 10;   // Kraken X60
    public static final int    kCANID_SteerRR        = 11;   // Kraken X60
    public static final int    kCANID_CANcoderRR     = 12;   // CANcoder

    public static final int    kCANID_Pigeon2        = 13;   // Pigeon2 IMU

    // RoboRIO CAN IDs
    public static final int    kCANID_WristRoller    = 15;   // Talon SRX - 775Pro
    public static final int    kCANID_WristRotary    = 16;   // Kraken X60
    public static final int    kCANID_WristCANcoder  = 17;   // CANcoder

    public static final int    kCANID_FeederRoller   = 19;   // Talon SRX - 775Pro
    public static final int    kCANID_FeederRotary   = 20;   // Kraken X60
    public static final int    kCANID_FeederCANcoder = 21;   // CANcoder

    public static final int    kCANID_ShooterLower   = 23;   // Kraken X60
    public static final int    kCANID_ShooterUpper   = 24;   // Kraken X60
    public static final int    kCANID_ShooterRotary  = 25;   // Kraken X60

    public static final int    kCANID_ElevatorLeft   = 27;   // Kraken X60
    public static final int    kCANID_ElevatorRight  = 28;   // Kraken X60

    public static final int    kCANID_CANdle         = 0;

    // Digital I/Os
    public static final int    kDIO_ElevatorDown     = 0; // REV Magnetic Limit Switch (inaccurate value)
  }

  /****************************************************************************
   * Wrist subsystem constants
   ****************************************************************************/
  public static final class WRConsts
  {
    /** Wrist roller modes */
    public enum WRRollerMode
    {
      STOP,    // Stop all rotation
      ACQUIRE, // Speed for acquiring a game piece
      EXPEL,   // Speed for expelling a game piece
      SHOOT,   // Speed for putting game piece into shooter 
      HANDOFF, // Speed for putting game piece into feeder 
      HOLD     // Maintain existing speed setting
    }
  }

  /****************************************************************************
   * Feeder subsystem constants
   ****************************************************************************/
  public static final class FDConsts
  {
    /** Feeder roller modes */
    public enum FDRollerMode
    {
      STOP,    // Stop all rotation
      SCORE,   // Speed for putting game piece into amp 
      HANDOFF, // Speed for handoff into feeder 
      HOLD     // Maintain existing speed setting
    }
  }

  /****************************************************************************
   * Shooter subsystem constants
   ****************************************************************************/
  public static final class SHConsts
  {
  }

  /****************************************************************************
   * Elevator subsystem constants
   ****************************************************************************/
  public static final class ELConsts
  {
  }

  /****************************************************************************
   * Vision (Limelight) constants
   ****************************************************************************/
  public static final class VIConsts
  {
    /** Field locations (poses) of AprilTags */
    public static final List<Pose2d> kAprilTagPoses = Collections.unmodifiableList(List.of(           // TODO: update for 2025 from Field layout and marking diagram
        new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0)),               // AprilTag ID: 0 (invalid)
        new Pose2d(new Translation2d(15.079472, 0.245872), Rotation2d.fromDegrees(120)),  // AprilTag ID: 1   - Blue source right
        new Pose2d(new Translation2d(16.185134, 0.883666), Rotation2d.fromDegrees(120)),  // AprilTag ID: 2   - Blue source left
        new Pose2d(new Translation2d(16.579342, 4.982718), Rotation2d.fromDegrees(180)),  // AprilTag ID: 3   - Red speaker right
        new Pose2d(new Translation2d(16.579342, 5.547868), Rotation2d.fromDegrees(180)),  // AprilTag ID: 4   - Red speaker center
        new Pose2d(new Translation2d(14.700758, 8.2042), Rotation2d.fromDegrees(270)),    // AprilTag ID: 5   - Red amp
        new Pose2d(new Translation2d(1.8415, 8.20426), Rotation2d.fromDegrees(270)),      // AprilTag ID: 6   - Blue amp
        new Pose2d(new Translation2d(-0.0381, 5.547868), Rotation2d.fromDegrees(0)),        // AprilTag ID: 7   - Blue speaker center
        new Pose2d(new Translation2d(-0.0381, 4.982718), Rotation2d.fromDegrees(0)),        // AprilTag ID: 8   - Blue speker left
        new Pose2d(new Translation2d(0.356108, 0.883666), Rotation2d.fromDegrees(60)),    // AprilTag ID: 9   - Red source right
        new Pose2d(new Translation2d(1.461516, 0.245872), Rotation2d.fromDegrees(60)),    // AprilTag ID: 10  - Red source left
        new Pose2d(new Translation2d(11.904726, 3.713226), Rotation2d.fromDegrees(300)),  // AprilTag ID: 11  - Red stage left
        new Pose2d(new Translation2d(11.904726, 4.49834), Rotation2d.fromDegrees(60)),    // AprilTag ID: 12  - Red stage right
        new Pose2d(new Translation2d(11.220196, 4.105148), Rotation2d.fromDegrees(180)),  // AprilTag ID: 13  - Red stage center
        new Pose2d(new Translation2d(5.320792, 4.105148), Rotation2d.fromDegrees(0)),     // AprilTag ID: 14  - Blue stage center
        new Pose2d(new Translation2d(4.641342, 4.49834), Rotation2d.fromDegrees(120)),    // AprilTag ID: 15  - Blue stage left
        new Pose2d(new Translation2d(4.641342, 3.713226), Rotation2d.fromDegrees(240))    // AprilTag ID: 16  - Blue stage right
    ));

    /** Destination field poses for the robot when using PathPlanner pathfinding */                   // TODO: update to desired 2025 field poses
    public static final Pose2d       kSpeakerPose   = new Pose2d(2.17, 4.49, Rotation2d.fromDegrees(-26));
    public static final Pose2d       kAmpPose       = new Pose2d(1.84, 7.77, Rotation2d.fromDegrees(-90));
  }

  /****************************************************************************
   * LED (CANdle) subsystem constants
   ****************************************************************************/
  public static final class LEDConsts
  {
    /** LED color to be used */
    public enum COLOR
    {
      OFF,      // CANdle off
      WHITE,    // CANdle white
      RED,      // CANdle red
      ORANGE,   // CANdle orange
      YELLOW,   // CANdle yellow
      GREEN,    // CANdle green
      BLUE,     // CANdle blue
      PURPLE,   // CANdle purple
      DASHBOARD // CANdle color taken from dashboard
    }

    /** LED animation to be used */
    public enum ANIMATION
    {
      COLORFLOW,  // Single color flow through string
      FIRE,       // Fire pattern from one end of string
      LARSON,     // Ping-pong pattern bouncing between string ends
      RAINBOW,    // Fading rainbow colors
      RGBFADE,    // Fading red, then green, then blue
      SINGLEFADE, // Fading with a single color
      STROBE,     // Strobe flashing with a single color
      TWINKLE,    // Twinkles leds on
      TWINKLEOFF, // Twinkles leds off
      CLEARALL,   // Clears animations
      DASHBOARD   // Animation taken from the dashboard
    }
  }

}
