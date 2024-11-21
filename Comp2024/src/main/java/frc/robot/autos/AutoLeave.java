
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LED;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoLeave extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Leave the starting zone while avoiding spike notes
   * 
   * @param ppAuto
   *          swerve drivetrain subsystem
   * @param drivetrain
   *          swerve drivetrain subsystem
   * @param led
   *          led subsystem
   */
  public AutoLeave(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain, LED led)
  {
    setName("AutoLeave");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(), "Leave zone"),
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
