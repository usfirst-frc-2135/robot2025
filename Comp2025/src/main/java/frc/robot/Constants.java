
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

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
  public static final String      kCompSN               = "03260A3A";
  public static final String      kPracticeSN           = "03238074";

  // Game controller definitions
  public static final int         kDriverPadPort        = 0;
  public static final int         kOperatorPadPort      = 1;

  public static final double      kStickDeadband        = 0.15;
  public static final double      kTriggerThreshold     = 0.25;

  public static final boolean     kRumbleOn             = true;
  public static final boolean     kRumbleOff            = false;
  public static final double      kRumbleIntensity      = 0.5;  // 0.0 is low, 1.0 is high

  // Phoenix firmware versions expected
  public static final int         kPhoenix6MajorVersion = 25;

  public static final String      kRobotString          = "RobotContainer";

  public static final String      kLLLeftName           = "limelight-left";
  public static final String      kLLRightName          = "limelight-right";

  // Robot physical dimensions

  public static final double      kBranchSpacing        = Units.inchesToMeters(13.0);  // Distance between branches
  public static final double      kL1Spacing            = Units.inchesToMeters(5.5);   // Distance to outside of branches
  public static final double      kRobotLength          = Units.inchesToMeters(34.5);  // Our robot length
  public static final double      kSetbackReefCoral     = kRobotLength / 2;                                       // Distance robot is set back from branch to score coral
  public static final double      kSetbackReefAlgae     = kRobotLength / 2 + Units.inchesToMeters(8.0);   // Distance robot is set back from reef to grab algae
  public static final double      kSetbackProcAlgae     = kRobotLength / 2 + Units.inchesToMeters(20.0);   // Distance robot is set back from reef to grab algae

  // Scoring poses relative to an AprilTag (X - robot setback, Y - left, center, right)

  public static final Transform2d kBranchCoralLeft      =
      new Transform2d(kSetbackReefCoral, -kBranchSpacing / 2, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralCenter    = new Transform2d(kSetbackReefCoral, 0, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralRight     =
      new Transform2d(kSetbackReefCoral, +kBranchSpacing / 2, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralLeftL1    =
      new Transform2d(kSetbackReefCoral, (-kBranchSpacing / 2) - kL1Spacing, Rotation2d.k180deg);
  public static final Transform2d kBranchCoralRightL1   =
      new Transform2d(kSetbackReefCoral, (+kBranchSpacing / 2) + kL1Spacing, Rotation2d.k180deg);

  public static final Transform2d kBranchAquireAlgae    = new Transform2d(kSetbackReefAlgae, 0, Rotation2d.k180deg);

  /****************************************************************************
   * CAN IDs and PWM IDs
   ****************************************************************************/
  public static final class Ports
  {
    public static final String kCANCarnivore        = "canivore1";
    public static final String kCANRio              = "rio";

    // CANivore CAN IDs - Swerve
    public static final int    kCANID_DriveLF       = 1;    // Kraken X60
    public static final int    kCANID_SteerLF       = 2;    // Kraken X60
    public static final int    kCANID_CANcoderLF    = 3;    // CANcoder

    public static final int    kCANID_DriveRF       = 4;    // Kraken X60     
    public static final int    kCANID_SteerRF       = 5;    // Kraken X60
    public static final int    kCANID_CANcoderRF    = 6;    // CANcoder

    public static final int    kCANID_DriveLR       = 7;    // Kraken X60
    public static final int    kCANID_SteerLR       = 8;    // Kraken X60
    public static final int    kCANID_CANcoderLR    = 9;    // CANcoder

    public static final int    kCANID_DriveRR       = 10;   // Kraken X60
    public static final int    kCANID_SteerRR       = 11;   // Kraken X60
    public static final int    kCANID_CANcoderRR    = 12;   // CANcoder

    public static final int    kCANID_Pigeon2       = 13;   // Pigeon2 IMU

    // RoboRIO CAN IDs
    public static final int    kCANID_ElevatorLeft  = 15;   // Kraken X60
    public static final int    kCANID_ElevatorRight = 16;   // Kraken X60

    public static final int    kCANID_WristRotary   = 18;   // Kraken X60 (Manipulator)
    public static final int    kCANID_WristCANcoder = 19;   // CANcoder   (Manipulator)

    public static final int    kCANID_ClawRoller    = 21;   // Kraken X60 (Manipulator)
    public static final int    kCANID_CoralDetector = 22;   // CANrange   (Manipulator)
    public static final int    kCANID_AlgaeDetector = 23;   // CANrange   (Manipulator)

    public static final int    kCANID_CANdle        = 0;

    // Digital I/Os
    public static final int    kDIO0_ElevatorDown   = 0;    // REV Magnetic Limit Switch
  }

  /****************************************************************************
   * Elevator subsystem constants
   ****************************************************************************/
  public static final class ELConsts
  {
    public static final String kReefLevelString = "ReefLevel";

    public enum ReefLevel
    {
      ONE,        // Coral Level One (Trough)
      TWO,        // Coral Level Two
      THREE,      // Coral Level Three
      FOUR,       // Coral Level Four
    }
  }

  /****************************************************************************
   * Manipulator subsystem constants
   ****************************************************************************/
  public static final class CRConsts // Claw roller
  {
    /** Manipulator claw roller modes */
    public enum ClawMode
    {
      STOP,             // Stop all rotation

      ALGAEACQUIRE,     // Speed for acquiring algae
      ALGAEHOLD,        // Speed for holding algae in claw
      ALGAEEXPEL,       // Speed for expelling algae
      ALGAESHOOT,       // Speed for shooting algae
      ALGAEPROCESSOR,   // Speed for putting algae into processor 
      ALGAEMAINTAIN,    // Maintain existing speed setting

      CORALACQUIRE,     // Speed for acquiring coral
      CORALEXPEL,       // Speed for expelling coral
      CORALMAINTAIN     // Maintain existing speed setting
    }
  }

  /****************************************************************************
   * Vision (Limelight) constants
   ****************************************************************************/
  public static final class VIConsts
  {
    public static final AprilTagFields      kGameField        = AprilTagFields.k2025ReefscapeWelded;
    public static final AprilTagFieldLayout kATField          = AprilTagFieldLayout.loadField(kGameField);

    public static final String              kReefBranchString = "ReefBranch";

    //** Reef branch used for alignment */
    public enum ReefBranch
    {
      LEFT(0), ALGAE(1), RIGHT(2);

      public final int value;

      private ReefBranch(int value)
      {
        this.value = value;
      }
    }

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
