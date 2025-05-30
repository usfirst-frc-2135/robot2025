package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Swerve drive under PID control to a goal pose
 */
public class SwervePIDController extends Command
{
  // Constants
  // private static final LinearVelocity           kMaxSpeed          = MetersPerSecond.of(3.5);     // Cap max applied velocity to 3.5 mps in either direction
  private static final Rotation2d               kRotationTolerance = Rotation2d.fromDegrees(2.0);
  private static final Distance                 kPositionTolerance = Inches.of(1.5);              // Was 0.8 inches which is tiny
  private static final LinearVelocity           kSpeedTolerance    = InchesPerSecond.of(2.0);    // Was 0.25 inches per second which is extremely small

  // Main objects
  public CommandSwerveDrivetrain                m_swerve;
  public Pose2d                                 m_goalPose;

  // PID controllers
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
      ntInst.getTable("swerve/PID").getDoubleTopic("vxMps").publish( );
  private DoublePublisher                       vyPub              =
      ntInst.getTable("swerve/PID").getDoubleTopic("vyMps").publish( );
  private DoublePublisher                       omegaPub           =
      ntInst.getTable("swerve/PID").getDoubleTopic("omegaRps").publish( );
  private DoublePublisher                       errorPub           =
      ntInst.getTable("swerve/PID").getDoubleTopic("error").publish( );

  /**
   * Swerve drive under PID control to a goal pose
   */
  public SwervePIDController(CommandSwerveDrivetrain swerve)
  {
    m_swerve = swerve;
    addRequirements(swerve);

    setName("SwervePID");
  }

  /**
   * Command factory
   */
  public static Command generateCommand(CommandSwerveDrivetrain swerve, Time timeout)
  {
    return new SwervePIDController(swerve).withTimeout(timeout);
  }

  @Override
  public void initialize( )
  {
    Pose2d currentPose = driveStatePose.get( );
    m_goalPose = m_swerve.findGoalPose(currentPose);
    DataLogManager.log(String.format("%s: initial current pose: %s goalPose %s", getName( ), currentPose, m_goalPose));
  }

  @Override
  public void execute( )
  {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState( );
    goalState.pose = m_goalPose;

    ChassisSpeeds speeds = m_DriveController.calculateRobotRelativeSpeeds(driveStatePose.get( ), goalState);

    vxPub.set(speeds.vxMetersPerSecond);
    vyPub.set(speeds.vyMetersPerSecond);
    omegaPub.set(speeds.omegaRadiansPerSecond);

    m_swerve.setControl(new SwerveRequest.ApplyRobotSpeeds( ).withSpeeds(speeds));
  }

  @Override
  public void end(boolean interrupted)
  {
    DataLogManager
        .log(String.format("%s: interrupted end conditions P: %s G: %s", getName( ), driveStatePose.get( ), m_goalPose));
  }

  @Override
  public boolean isFinished( )
  {
    Pose2d diff = driveStatePose.get( ).relativeTo(m_goalPose);

    errorPub.set(Math.sqrt(Math.pow(diff.getX( ), 2) + Math.pow(diff.getY( ), 2)));

    boolean rotation = MathUtil.isNear(0.0, diff.getRotation( ).getRotations( ), kRotationTolerance.getRotations( ), 0.0, 1.0);

    boolean position = diff.getTranslation( ).getNorm( ) < kPositionTolerance.in(Meters);

    boolean speed = driveSpeeds.get( ).vxMetersPerSecond < kSpeedTolerance.in(MetersPerSecond)
        && driveSpeeds.get( ).vyMetersPerSecond < kSpeedTolerance.in(MetersPerSecond);

    DataLogManager.log(String.format("%s: end conditions R: %s P: %s S: %s", getName( ), rotation, position, speed));

    endCondition = endDebouncer.calculate(rotation && position && speed);

    endConditionLogger.accept(endCondition);

    return endCondition;
  }
}
