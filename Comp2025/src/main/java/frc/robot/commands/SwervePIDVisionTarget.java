package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VIConsts;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Swerve drive under PID control to a goal pose
 */
public class SwervePIDVisionTarget extends Command
{
  // Constants
  private static final LinearVelocity           kMaxSpeed          = MetersPerSecond.of(3.5);     // Cap max applied velocity to 3.5 mps in either direction
  private static final Rotation2d               kRotationTolerance = Rotation2d.fromDegrees(2.0);
  private static final Distance                 kPositionTolerance = Inches.of(1.5);              // Was 0.8 inches which is tiny
  private static final LinearVelocity           kSpeedTolerance    = InchesPerSecond.of(2.0);    // Was 0.25 inches per second which is extremely small

  // Main objects
  private CommandSwerveDrivetrain               m_swerve;
  private Vision                                m_vision;
  private Pose2d                                m_goalPose; // NOTE: Don't need goal pose tracking

  // PID controllers - NOTE: Our PID controllers are in the vision class, so we don't need these // Was 5.0 mps for a 1 m offset (too large)
  private static final PIDConstants             kTranslationPID    = new PIDConstants(5.0, 0, 0); // Was 5.0 mps for a 1 m offset (too large)
  private static final PIDConstants             kRotationPID       = new PIDConstants(5.0, 0, 0);
  private PPHolonomicDriveController            m_DriveController  =
      new PPHolonomicDriveController(kTranslationPID, kRotationPID);

  // Debouncer debouncer
  private static final Time                     kEndDebounce       = Seconds.of(0.04);
  private final Debouncer                       endDebouncer       =
      new Debouncer(kEndDebounce.in(Seconds), Debouncer.DebounceType.kBoth);
  private final BooleanPublisher                endConditionLogger =
      ntInst.getTable("swerve").getBooleanTopic("PIDEndCondition").publish( );
  private boolean                               endCondition       = false;

  // Network tables entries
  private static final NetworkTableInstance     ntInst             = NetworkTableInstance.getDefault( );
  private static final NetworkTable             driveStateTable    = ntInst.getTable("DriveState");
  private static final StructSubscriber<Pose2d> driveStatePose     =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d( ));
  private final StructSubscriber<ChassisSpeeds> driveSpeeds        =
      driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).subscribe(new ChassisSpeeds( ));

  private DoublePublisher                       vxPub              =
      ntInst.getTable("swerve/PIDVisionTarget").getDoubleTopic("vxMps").publish( );
  private DoublePublisher                       vyPub              =
      ntInst.getTable("swerve/PIDVisionTarget").getDoubleTopic("vyMps").publish( );
  private DoublePublisher                       omegaPub           =
      ntInst.getTable("swerve/PIDVisionTarget").getDoubleTopic("omegaRps").publish( );
  private DoublePublisher                       errorPub           =
      ntInst.getTable("swerve/PIDVisionTarget").getDoubleTopic("error").publish( );

  private static final NetworkTable             robotTable         = ntInst.getTable(Constants.kRobotString);
  private static final IntegerSubscriber        reefBranch         =
      robotTable.getIntegerTopic(VIConsts.kReefBranchString).subscribe((0));

  /**
   * Swerve drive under PID control to a goal pose
   */
  public SwervePIDVisionTarget(CommandSwerveDrivetrain swerve, Vision vision)
  {
    // TODO: Since we will be commanding the swerve subsystem to drive to where we want to go,
        // TODO: we MUST pass in a reference to call setControl with the desired speeds
        // TODO: I uncommented where you had commented it out, since it needs to come back
    CommandSwerveDrivetrain m_swerve = swerve;
    m_vision = vision;
    addRequirements(swerve);
    setName("SwervePIDVisionTarget");
  }

  /**
   * Command factory
   */
  public static Command generateCommand(CommandSwerveDrivetrain swerve, Vision vision, Time timeout)
  {
    return new SwervePIDVisionTarget(swerve, vision).withTimeout(timeout);
  }

  @Override
  public void initialize( )
  {

    // TODO: update the log message to print the reefBranch in use
    // TODO: it will let us choose left or right limelight to use to align with
    // TODO: but won't use the pose but leave log message
    // Pose2d currentPose = driveStatePose.get( );
    // m_goalPose = Vision.findGoalPose(currentPose);

    // TODO: I think when we use the IntegerSubscriber, we need to use reefBranch.get() below (and anywhere else)
    DataLogManager.log(String.format("%s: initial reefBranch: %s ", getName( ), reefBranch));
  }

  @Override
  public void execute( )
  {
    // TODO: Use the member variable holding the reefBranch (left or right)
    // TODO: Call the aimProportional and rangeProportional methods in the vision calls and pass in the left/right (or center) variable
    // TODO:    these should return the translation and angular velocit speeds that get put into the setControl
    // TODO: Remove the lines of code that get the goal pose, calculate the speeds, publish them
    // TODO: Comment out the swerve drive (but KEEP) setControl call until we can update it with the correct one
    // need to get tx and ty from proper camera, use aim proportional and range proportional methods to get chassis speeds, will go into setControl
    // PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState( );
    // goalState.pose = m_goalPose;
    // ChassisSpeeds speeds = m_DriveController.calculateRobotRelativeSpeeds(driveStatePose.get( ), goalState);
    // vxPub.set(speeds.vxMetersPerSecond);
    // vyPub.set(speeds.vyMetersPerSecond);
    // omegaPub.set(speeds.omegaRadiansPerSecond);

    // TODO: We can't create a new vision class here, but we can pass it in when the class is created (like the swerve subsystem) in m_vision
    Vision m_rangeProportionall = new Vision( );
    // TODO: Use the kMaxSpeed constant above to pass into ...rangeProportional
    LinearVelocity maxSpeed;
    // TODO: declare a LinearVelocity variable "speed" and set it to the return value from ...rangeProportional which returns a speed
    //*m_rangeProportionall.rangeProportional(maxSpeed);

    // TODO: We can't create a new vision class here, but we can pass it in when the class is created (like the swerve subsystem) in m_vision
    Vision m_aimProportionall = new Vision( );
    // TODO: Use the kMaxAngularRate constant above to pass into ...aimProportional
    AngularVelocity kMaxAngularRate;
    // TODO: declare an AngularVelocity variable "rps" and set it to the return value from ...aimProportional which returns rotations per second
    //*m_aimProportionall.aimProportional(kMaxAngularRate);

    // TODO: We need a new m_swerve method that will take the linear speed (in chassis X direction) and rotations per second (like Apollo's code)
    // m_swerve.setControl(new SwerveRequest.ApplyRobotSpeeds( ).withSpeeds(speeds));
  }

  private Vision m_vision( )
  {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'm_vision'");
  }

  @Override
  public void end(boolean interrupted)
  {
    // TODO: Update this logging call to make sense for our use - display the left/right, and end speeds
    // TODO: print what tx and ty are
    DataLogManager.log(String.format("%s: interrupted end conditions for SwervePIDVisionTarget L: %s R: %s EndSpeeds: %s",
        getName( ), driveStatePose.get( ), m_goalPose));
  }

  @Override
  public boolean isFinished( )
  {
    // TODO: We'll need a new vision method that 'does the work' to return whether tx/ty are close enough to the desired location
    // TODO: Remove all of the pose references, keep a log message that prints only when the command is done
    // TODO: Our end condition will be the vision check plus the drive speed checks below
    //tx and ty are where we want them to be

    //*errorPub.set(Math.sqrt(Math.pow(diff.getX( ), 2) + Math.pow(diff.getY( ), 2)));

    //*boolean rotation = MathUtil.isNear(0.0, diff.getRotation( ).getRotations( ), kRotationTolerance.getRotations( ), 0.0, 1.0);

    //*boolean position = diff.getTranslation( ).getNorm( ) < kPositionTolerance.in(Meters);

    boolean speed = driveSpeeds.get( ).vxMetersPerSecond < kSpeedTolerance.in(MetersPerSecond)
        && driveSpeeds.get( ).vyMetersPerSecond < kSpeedTolerance.in(MetersPerSecond);

    DataLogManager.log(String.format("%s: end conditions for SwervePIDVisionTarget S: %s", getName( ), speed));

    endCondition = endDebouncer.calculate(speed);

    endConditionLogger.accept(endCondition);

    DataLogManager.log(String.format("End condition: ", endCondition));

    return endCondition;
  }
}
