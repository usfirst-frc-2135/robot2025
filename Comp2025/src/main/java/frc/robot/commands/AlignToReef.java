
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A command that logs a string when initialized.
 */
public class AlignToReef extends InstantCommand
{
  private CommandSwerveDrivetrain m_swerve;
  private Pose2d                  m_targetPose;
  private final PathConstraints   kPathFindConstraints = new PathConstraints( // 
      3.5,       // kMaxVelocityMps                               (slowed from 3.0 for testing)    
      3.5, // kMaxAccelerationMpsSq                         (slowed from 3.0 for testing)  
      2.0 * Math.PI,            // kMaxAngularSpeedRadiansPerSecond 
      2.0 * Math.PI             // kMaxAngularSpeedRadiansPerSecondSquared 
  );

  /**
   * Creates a new a AlignToReef command
   * 
   * @param swerve
   *          swerve subsystem
   */
  public AlignToReef(CommandSwerveDrivetrain swerve)
  {
    m_swerve = swerve;
    m_targetPose = m_swerve.findTargetPose( );
  }

  @Override
  public void initialize( )
  {

    DataLogManager.log(String.format("AlignToReef: ReefLevel %d ReefBranch %d target %s", 3, 0, m_targetPose));                                        //
    AutoBuilder.pathfindToPose(m_targetPose, kPathFindConstraints, 0.0);            //
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return true;
  }
}
