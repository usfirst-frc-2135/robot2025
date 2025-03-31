
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoLeave extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Leave the starting line
   * 
   * @param ppAuto
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoLeave(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain)
  {
    setName("AutoLeave");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Leave starting line"),
        drivetrain.getPathCommand(ppAuto.get(0))
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
