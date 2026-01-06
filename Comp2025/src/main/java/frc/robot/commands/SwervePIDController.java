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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.subsystems.Vision;

/**
 * Swerve drive under PID control to a goal pose
 */
public class SwervePIDController extends Command
{
  // Constants
  private static final double                   kMaxVelocity       = 3.8;                                 // Max robot velocity in MPS
  private static final double                   kMaxAcceleration   = 4.0;                                 // Max robot acceleration in MPS^2
  private static final Rotation2d               kRotationTolerance = Rotation2d.fromDegrees(1.0);
  private static final Distance                 kPositionTolerance = Inches.of(1.5);
  private static final LinearVelocity           kSpeedTolerance    = InchesPerSecond.of(2.0);

  // Main objects
  private CommandSwerveDrivetrain               m_swerve;
  private Pose2d                                m_goalPose;

  // PID controllers
  private static final PIDConstants             kTranslationPID    = new PIDConstants(2.6, 0, 0);
  private static final PIDConstants             kRotationPID       = new PIDConstants(10.0, 0, 0);
  private PPHolonomicDriveController            m_DriveController  =
      new PPHolonomicDriveController(kTranslationPID, kRotationPID);
  private static final SlewRateLimiter          m_vxAccelLimiter   = new SlewRateLimiter(kMaxAcceleration);
  private static final SlewRateLimiter          m_vyAccelLimiter   = new SlewRateLimiter(kMaxAcceleration);

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
    m_goalPose = Vision.findGoalPose(currentPose);

    // Initialize the slew rate limiter
    m_vxAccelLimiter.reset(driveSpeeds.get( ).vxMetersPerSecond);
    m_vyAccelLimiter.reset(driveSpeeds.get( ).vyMetersPerSecond);

    DataLogManager.log(String.format("%s: initial current pose: %s goalPose %s", getName( ), currentPose, m_goalPose));
  }

  @Override
  public void execute( )
  {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState( );
    goalState.pose = m_goalPose;

    // Calculate theoretical PID chassis speeds to get to our goal (assumes no max velocity or acceleration)
    ChassisSpeeds speeds = m_DriveController.calculateRobotRelativeSpeeds(driveStatePose.get( ), goalState);

    // Constrain the speeds (velocities) to what the robot can actually do
    speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -kMaxVelocity, kMaxVelocity);
    speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -kMaxVelocity, kMaxVelocity);

    // Constrain the acceleration to what the robot can actually do (with no/little wheel slip)
    speeds.vxMetersPerSecond = m_vxAccelLimiter.calculate(speeds.vxMetersPerSecond);
    speeds.vyMetersPerSecond = m_vyAccelLimiter.calculate(speeds.vyMetersPerSecond);

    // Output for logging
    vxPub.set(speeds.vxMetersPerSecond);
    vyPub.set(speeds.vyMetersPerSecond);
    omegaPub.set(speeds.omegaRadiansPerSecond);

    m_swerve.setControl(new SwerveRequest.ApplyRobotSpeeds( ).withSpeeds(speeds));
    // m_swerve.driveRobotRelative(speeds);     // TODO: swerve setpoint generator testing
  }

  @Override
  public void end(boolean interrupted)
  {
    // Reset logging outputs
    vxPub.set(0.0);
    vyPub.set(0.0);
    omegaPub.set(0.0);

    DataLogManager.log(String.format("%s: interrupted: %s end conditions P: %s G: %s", getName( ), interrupted,
        driveStatePose.get( ), m_goalPose));
  }

  @Override
  public boolean isFinished( )
  {
    Transform2d poseError = new Transform2d(driveStatePose.get( ), m_goalPose);

    double error = Math.hypot(poseError.getX( ), poseError.getY( ));
    errorPub.set(error);

    boolean position = error < kPositionTolerance.in(Meters);

    boolean rotation = MathUtil.isNear(m_goalPose.getRotation( ).getRotations( ),
        driveStatePose.get( ).getRotation( ).getRotations( ), kRotationTolerance.getRotations( ), 0.0, 1.0);
    // DataLogManager.log(String.format("exp: %s act: %s tol: %s", m_goalPose.getRotation( ).getRotations( ),
    //     driveStatePose.get( ).getRotation( ).getRotations( ), kRotationTolerance.getRotations( )));

    boolean speed = Math.abs(driveSpeeds.get( ).vxMetersPerSecond) < kSpeedTolerance.in(MetersPerSecond)
        && Math.abs(driveSpeeds.get( ).vyMetersPerSecond) < kSpeedTolerance.in(MetersPerSecond);

    DataLogManager.log(String.format("%s: end conditions R: %s P: %s S: %s", getName( ), rotation, position, speed));

    endCondition = endDebouncer.calculate(rotation && position && speed);

    endConditionLogger.accept(endCondition);

    return endCondition;
  }
}
