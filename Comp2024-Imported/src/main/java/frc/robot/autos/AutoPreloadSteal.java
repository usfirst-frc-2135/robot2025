
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.Robot;
import frc.robot.commands.AcquireNote;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PassNote;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 * Auto command that shoots preloaded note and feeds notes from centerline
 */
public class AutoPreloadSteal extends SequentialCommandGroup
{
  /**
   * Autonomous command to: // TODO: This is not yet implemented below
   * 1a - Drive to a scoring position
   * 1b - Shoot the preloaded note
   * 2a - Drive to get a centerline note
   * 2b - Drive to feeding position
   * 2c - Feed the note
   * 3a - Drive to get a centerline note
   * 3b - Drive to feeding position
   * 3c - Feed the note
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   * @param intake
   *          intake subsystem
   * @param shooter
   *          shooter subsystem
   * @param led
   *          led subsystem
   */
  public AutoPreloadSteal(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter,
      LED led, HID hid)
  {
    setName("AutoPreloadSteal");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new InstantCommand(()->Robot.timeMarker(getName())),

        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getPathCommand(ppPaths.get(0)),

        new LogCommand(getName(), "Score preloaded note"),
        new PassNote(shooter, intake, led),

        new LogCommand(getName(), "Drive to centerline acquire note 1"),
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(
            drivetrain.getPathCommand(ppPaths.get(1)),
            drivetrain.getPathCommand(ppPaths.get(2))
          ),
          new AcquireNote(intake, led, hid)
        ),
        new ConditionalCommand(
          new PassNote(shooter, intake, led),
          new LogCommand(getName(), "Missed centerline 1 note"),
          intake::isNoteDetected
        ),

        new LogCommand(getName(), "Drive to centerline acquire note 2"),
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(
            drivetrain.getPathCommand(ppPaths.get(3)),
            drivetrain.getPathCommand(ppPaths.get(4))
          ),
          new AcquireNote(intake, led, hid)
        ),
        new ConditionalCommand(
          new PassNote(shooter, intake, led),
          new LogCommand(getName(), "Missed centerline 2 note"),
          intake::isNoteDetected
        ),

        new LogCommand(getName(), "Turn off intake rollers"), 
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getCurrentPosition),

        new InstantCommand(()->Robot.timeMarker(getName()))

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
