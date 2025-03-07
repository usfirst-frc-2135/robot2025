package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireAlgae;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreAlgae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Auto command that can be used for testing new sequences
 */
public class AutoPreloadAlgae2 extends SequentialCommandGroup
{
    /**
     * Autonomous command to:
     * 1 - Run an auto with a test path
     * 
     * @param ppPaths
     *            list of auto paths to follow
     * @param drivetrain
     *            swerve drivetrain subsystem
     */
    public AutoPreloadAlgae2(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
            Manipulator manipulator, LED led, HID hid)
    {
        setName("AutoPreloadAlgae2");

        addCommands(
                // Add Commands here:

                // @formatter:off

        new LogCommand(getName(), "Drive to reef and score preloaded coral"),
        drivetrain.getPathCommand(ppPaths.get(0)),
        new ScoreCoral(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to face and acquire algae"),
        drivetrain.getPathCommand(ppPaths.get(1)),
        new AcquireAlgae(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to proc/net score algae"),
        drivetrain.getPathCommand(ppPaths.get(2)),
        new ScoreAlgae(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to face and acquire algae"),
        drivetrain.getPathCommand(ppPaths.get(3)),
        new AcquireAlgae(elevator, manipulator, led, hid),

        new LogCommand(getName(), "Drive to net and score algae"),
        drivetrain.getPathCommand(ppPaths.get(4)),
        new ScoreAlgae(elevator, manipulator, led, hid)


        //new ScoreCoral(elevator, manipulator, led, hid)
        
        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
