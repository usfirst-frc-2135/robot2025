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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DrivePIDCommand extends Command
{
  public CommandSwerveDrivetrain      mSwerve;
  public final Pose2d                 goalPose;

  public static final PIDConstants    kTranslationPID     = new PIDConstants(5.0, 0, 0);
  public static final PIDConstants    kRotationPID        = new PIDConstants(5.0, 0, 0);
  private PPHolonomicDriveController  mDriveController    = new PPHolonomicDriveController(kTranslationPID, kRotationPID);

  public static final Rotation2d      kRotationTolerance  = Rotation2d.fromDegrees(2.0);
  public static final Distance        kPositionTolerance  = Inches.of(0.8);
  public static final LinearVelocity  kSpeedTolerance     = InchesPerSecond.of(0.25);

  public static final Time            kEndTriggerDebounce = Seconds.of(0.04);

  public static final PathConstraints kPathConstraints    = new PathConstraints(1.25, 1.25, 1 / 2 * Math.PI, 1 * Math.PI); // The constraints for this path.

  private final Trigger               endTrigger;
  private final Trigger               endTriggerDebounced;

  private final BooleanPublisher      endTriggerLogger    =
      NetworkTableInstance.getDefault( ).getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish( );

  public DrivePIDCommand(CommandSwerveDrivetrain mSwerve, Pose2d goalPose)
  {
    this.mSwerve = mSwerve;
    this.goalPose = goalPose;

    setName("DrivePIDCommand");

    DataLogManager.log(String.format("%s: constructor ....................", getName( )));

    endTrigger = new Trigger(( ) ->
    {
      Pose2d diff = mSwerve.getState( ).Pose.relativeTo(goalPose);

      var rotation = MathUtil.isNear(0.0, diff.getRotation( ).getRotations( ), kRotationTolerance.getRotations( ), 0.0, 1.0);

      var position = diff.getTranslation( ).getNorm( ) < kPositionTolerance.in(Meters);

      var speed = mSwerve.getState( ).Speeds.vxMetersPerSecond < kSpeedTolerance.in(MetersPerSecond)
          && mSwerve.getState( ).Speeds.vyMetersPerSecond < kSpeedTolerance.in(MetersPerSecond);

      DataLogManager.log(String.format("%s: end trigger conditions R: %s P: %s S: %s", getName( ), rotation, position, speed));

      return rotation && position && speed;
    });

    endTriggerDebounced = endTrigger.debounce(kEndTriggerDebounce.in(Seconds));
  }

  public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d goalPose, Time timeout)
  {
    DataLogManager.log(String.format("DrivePIDCommand: generateCommand ...................."));
    return new DrivePIDCommand(swerve, goalPose).withTimeout(timeout)
        .finallyDo(( ) -> swerve.setControl(new SwerveRequest.SwerveDriveBrake( )));
  }

  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: initialize ....................", getName( )));

    endTriggerLogger.accept(endTrigger.getAsBoolean( ));
  }

  @Override
  public void execute( )
  {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState( );
    goalState.pose = goalPose;

    // DataLogManager.log(String.format("%s: execute ....................", getName( )));

    endTriggerLogger.accept(endTrigger.getAsBoolean( ));

    ChassisSpeeds speeds = mDriveController.calculateRobotRelativeSpeeds(mSwerve.getState( ).Pose, goalState);

    DataLogManager.log(String.format("%s: execute speed %s", getName( ), speeds));

    mSwerve.setControl(new SwerveRequest.ApplyRobotSpeeds( ).withSpeeds(speeds));
  }

  @Override
  public void end(boolean interrupted)
  {
    DataLogManager
        .log(String.format("%s: interrupted end trigger conditions G: %s P: %s", getName( ), goalPose, mSwerve.getState( ).Pose));

    endTriggerLogger.accept(endTrigger.getAsBoolean( ));
  }

  @Override
  public boolean isFinished( )
  {
    DataLogManager.log(String.format("%s: isFinished ....................", getName( )));
    return endTriggerDebounced.getAsBoolean( );
  }
}
