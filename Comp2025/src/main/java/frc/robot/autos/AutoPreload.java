
package frc.robot.autos;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ExpelCoral;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Manipulator;

/**
 * Auto command that scores just the preloaded coral
 */
public class AutoPreload extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a branch
   * 2 - Score a preloaded coral
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoPreload(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
      Manipulator manipulator, HID hid, Supplier<Command> getReefLevelCommand)
  {
    setName("AutoPreload");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Select scoring level"),
        getReefLevelCommand.get(),

        new LogCommand(getName(), "Drive to branch and score preload coral"),
        drivetrain.getPathCommand(ppPaths.get(0)),
        new ScoreCoral(elevator, manipulator),
        new ExpelCoral(elevator, manipulator)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
