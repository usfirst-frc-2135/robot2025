
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.Robot;
import frc.robot.commands.AcquireNote;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreSpeaker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

/**
 * Auto command that shoots preloaded note and scores all spike notes
 */
public class AutoScore4 extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a scoring position
   * 1a - Shoot the preloaded note
   * 2a - Deploy intake and run rollers
   * 2b - Drive to a spike and back while acquiring a note
   * 2c - Shoot a the first spike note
   * 3a - Deploy intake and run rollers
   * 3b - Drive to a spike and back while acquiring a note
   * 3c - Shoot a the second spike note
   * 4a - Deploy intake and run rollers
   * 4b - Drive to a spike and back while acquiring a note
   * 4c - Shoot a the third spike note
   * 
   * @param ppPaths
   *          swerve drivetrain subsystem
   * @param drivetrain
   *          swerve drivetrain subsystem
   * @param intake
   *          intake subsystem
   * @param shooter
   *          shooter subsystem
   * @param led
   *          led subsystem
   */
  public AutoScore4(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, LED led,
      HID hid)
  {
    setName("AutoScore4");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new InstantCommand(()->Robot.timeMarker(getName())),

        new LogCommand(getName(), "Drive to scoring pose"),
        drivetrain.getPathCommand(ppPaths.get(0)),

        new LogCommand(getName(), "Score preloaded note"),
        new ScoreSpeaker(shooter, intake, led),

        new LogCommand(getName(), "Deploy intake before moving"),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.ACQUIRE, intake::getIntakeDeployed),

        new LogCommand(getName(), "Drive to first spike while intaking"),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            drivetrain.getPathCommand(ppPaths.get(1)),
            drivetrain.getPathCommand(ppPaths.get(2))
          ),
          new AcquireNote(intake, led, hid)
        ),
        new ConditionalCommand(
          new ScoreSpeaker(shooter, intake, led),
          new LogCommand(getName(), "Missed first spike note"),
          intake::isNoteDetected
        ),

        new LogCommand(getName(), "Drive to second spike while intaking"),
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(
            drivetrain.getPathCommand(ppPaths.get(3)),
            drivetrain.getPathCommand(ppPaths.get(4))
          ),
          new AcquireNote(intake, led, hid)
        ),
        new ConditionalCommand(
          new ScoreSpeaker(shooter, intake, led),
          new LogCommand(getName(), "Missed second spike note"),
          intake::isNoteDetected
        ),

        new LogCommand(getName(), "Drive to third spike while intaking"),
        new ParallelDeadlineGroup( 
          new SequentialCommandGroup(
            drivetrain.getPathCommand(ppPaths.get(5)),
            drivetrain.getPathCommand(ppPaths.get(6))
          ),
          new AcquireNote(intake, led, hid)
        ),
        new ConditionalCommand(
          new ScoreSpeaker(shooter, intake, led),
          new LogCommand(getName(), "Missed third spike note "),
          intake::isNoteDetected
        ),
          
        new LogCommand(getName(), "Turn off intake rollers"), 
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getCurrentPosition),

        new LogCommand(getName(), "Drive to c3 while intaking"),

        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppPaths.get(7)),
          new AcquireNote(intake,led,hid)
        ),
 
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
