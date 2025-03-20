
package frc.robot.autos;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AcquireCoral;
import frc.robot.commands.ExpelCoral;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Auto command that scores the preloaded coral and then one more
 */
public class AutoPreloadCoral extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a branch
   * 2 - Score a preloaded coral
   * 3 - Drive to coral station
   * 4 - Acquire a second coral
   * 5 - Drive to a branch
   * 6 - Score the second coral
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoPreloadCoral(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
      Manipulator manipulator, LED led, HID hid, Supplier<Command> getReefLevelCommand)
  {
    setName("AutoPreloadCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Auto Start")),

        new LogCommand(getName(), "Select scoring level"),
        getReefLevelCommand.get(),

        
        new LogCommand(getName(), "Drive to branch and score preload coral"),
        drivetrain.getPathCommand(ppPaths.get(0)),
        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Drive to branch")), 

        
        new ScoreCoral(elevator, manipulator, led, hid),
        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Score Coral 1")),  

        new ExpelCoral(elevator, manipulator, led, hid),
        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Expel Coral 1")),  

        new LogCommand(getName(), "Drive to coral station and acquire second coral"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppPaths.get(1)),
          new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Drive to coral station")),  

          new AcquireCoral(elevator, manipulator, led, hid),
          new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Acquire Coral 2"))
        ), 

        new LogCommand(getName(), "Drive to branch and score second coral"),
        drivetrain.getPathCommand(ppPaths.get(2)),
        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Drive to branch")),

        new ScoreCoral(elevator, manipulator, led, hid),
        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Score Coral 2")),

        new ExpelCoral(elevator, manipulator, led, hid),
        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Expel Coral 2")),

        new InstantCommand(( ) -> Robot.timeMarker("AutoPreloadCoral: Auto End"))
                
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
