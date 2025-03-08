package frc.robot.autos;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireCoral;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Auto command that scores the preloaded coral and then two more
 */
public class AutoPreloadCoral2 extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a branch
   * 2 - Score a preloaded coral
   * 3 - Drive to coral station
   * 4 - Acquire a second coral
   * 5 - Drive to a branch
   * 6 - Score the second coral
   * 7 - Drive to coral station
   * 8 - Acquire a third coral
   * 9 - Drive to a branch
   * 10 - Score the third coral
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoPreloadCoral2(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
      Manipulator manipulator, LED led, HID hid, Supplier<Command> getReefLevelCommand)
  {
    setName("AutoPreloadCoral2");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Select scoring level"),
        getReefLevelCommand.get(),

        new LogCommand(getName(), "Drive to branch and score preload coral"),
        drivetrain.getPathCommand(ppPaths.get(0)),
        // new ScoreCoral(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to coral station and acquire second coral"),
        drivetrain.getPathCommand(ppPaths.get(1)),
        // new AcquireCoral(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to branch and score second coral"),
        drivetrain.getPathCommand(ppPaths.get(2)),
        // new ScoreCoral(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to coral station and acquire third coral"),
        drivetrain.getPathCommand(ppPaths.get(3)),
        // new AcquireCoral(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to branch and score third coral"),   
        drivetrain.getPathCommand(ppPaths.get(4))
        // new ScoreCoral(elevator, manipulator, led, hid)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
