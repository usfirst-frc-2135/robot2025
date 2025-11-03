package frc.robot.autos;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireAlgae;
import frc.robot.commands.ExpelCoral;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Manipulator;

/**
 * Auto command that scores the preloaded coral and scores an algae
 */
public class AutoPreloadAlgae extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a branch
   * 2 - Score a preloaded coral
   * 3 - Drive to an algae face
   * 4 - Acquire the algae
   * 5 - Drive to the processor
   * 6 - Score the algae
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoPreloadAlgae(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
      Manipulator manipulator, HID hid, Supplier<Command> getReefLevelCommand)
  {
    setName("AutoPreloadAlgae");

    addCommands(
        // Add Commands here:

        // @formatter:off
                
        new LogCommand(getName(), "Select scoring level"),
        getReefLevelCommand.get(),

        new LogCommand(getName(), "Drive to branch and score preload coral"),
        drivetrain.getPathCommand(ppPaths.get(0)),
        new ScoreCoral(elevator, manipulator),
        new ExpelCoral(elevator, manipulator),

        new LogCommand(getName(), "Drive to face and acquire algae"),
        drivetrain.getPathCommand(ppPaths.get(1)),
        new AcquireAlgae(elevator, manipulator, hid),

        new LogCommand(getName(), "Drive to proc/net and score algae"),
        drivetrain.getPathCommand(ppPaths.get(2)),
        new ScoreAlgae(elevator, manipulator) 
               
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
