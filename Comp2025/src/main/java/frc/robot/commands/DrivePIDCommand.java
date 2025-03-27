package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DrivePIDCommand extends Command
{
  public CommandSwerveDrivetrain      m_drivetrain;
  public final Pose2d                 m_goalPose;

  public static final PIDConstants    kTranslationPID     = new PIDConstants(5.0, 0, 0);
  public static final PIDConstants    kRotationPID        = new PIDConstants(5.0, 0, 0);
  private PPHolonomicDriveController  mDriveController    = new PPHolonomicDriveController(kTranslationPID, kRotationPID);

  public static final Rotation2d      kRotationTolerance  = Rotation2d.fromDegrees(2.0);
  public static final Distance        kPositionTolerance  = Inches.of(0.8);
  public static final LinearVelocity  kSpeedTolerance     = InchesPerSecond.of(0.25);

  public static final Time            kEndTriggerDebounce = Seconds.of(0.04);

  public static final PathConstraints kPathConstraints    = new PathConstraints(1.25, 1.25, 1 / 2 * Math.PI, 1 * Math.PI); // The constraints for this path.

  private final Debouncer             endDebouncer        =
      new Debouncer(kEndTriggerDebounce.in(Seconds), Debouncer.DebounceType.kBoth);
  private final BooleanPublisher      endConditionLogger  =
      NetworkTableInstance.getDefault( ).getTable("logging").getBooleanTopic("SwervePIDEndCondition").publish( );

  private boolean                     endCondition        = false;

  public DrivePIDCommand(CommandSwerveDrivetrain drivetrain, Pose2d goalPose)
  {
    this.m_drivetrain = drivetrain;
    this.m_goalPose = goalPose;

    setName("DrivePIDCommand");
  }

  public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d m_goalPose, Time timeout)
  {
    return new DrivePIDCommand(swerve, m_goalPose).withTimeout(timeout)
        .finallyDo(( ) -> swerve.setControl(new SwerveRequest.SwerveDriveBrake( )));
  }

  @Override
  public void initialize( )
  {}

  @Override
  public void execute( )
  {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState( );
    goalState.pose = m_goalPose;

    ChassisSpeeds speeds = mDriveController.calculateRobotRelativeSpeeds(m_drivetrain.getState( ).Pose, goalState);

    m_drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds( ).withSpeeds(speeds));
  }

  @Override
  public void end(boolean interrupted)
  {
    DataLogManager.log(String.format("%s: interrupted end trigger conditions G: %s P: %s", getName( ), m_goalPose,
        m_drivetrain.getState( ).Pose));
  }

  @Override
  public boolean isFinished( )
  {
    Pose2d diff = m_drivetrain.getState( ).Pose.relativeTo(m_goalPose);

    boolean rotation = MathUtil.isNear(0.0, diff.getRotation( ).getRotations( ), kRotationTolerance.getRotations( ), 0.0, 1.0);

    boolean position = diff.getTranslation( ).getNorm( ) < kPositionTolerance.in(Meters);

    boolean speed = m_drivetrain.getState( ).Speeds.vxMetersPerSecond < kSpeedTolerance.in(MetersPerSecond)
        && m_drivetrain.getState( ).Speeds.vyMetersPerSecond < kSpeedTolerance.in(MetersPerSecond);

    DataLogManager.log(String.format("%s: end trigger conditions R: %s P: %s S: %s", getName( ), rotation, position, speed));

    endCondition = endDebouncer.calculate(rotation && position && speed);

    endConditionLogger.accept(endCondition);

    return endCondition;
  }
}
