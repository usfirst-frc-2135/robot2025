
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Auto command that can be used for testing new sequences
 */
public class AutoTest extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Run an auto with a test path
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoTest(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain)
  {
    setName("AutoTest");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Drive a test path"),
        drivetrain.getModulePositionsCommand(false),
        drivetrain.getPathCommand(ppPaths.get(0)),
        drivetrain.getModulePositionsCommand(true)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
