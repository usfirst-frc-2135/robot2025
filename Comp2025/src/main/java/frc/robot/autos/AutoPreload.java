
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.commands.ScoreCoral;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

/**
 * Auto command that can be used for testing new sequences
 */
public class AutoPreload extends SequentialCommandGroup
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
    public AutoPreload(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain)
    {
        setName("AutoPreload");

        addCommands(
                // Add Commands here:

                // @formatter:off

        new LogCommand(getName(), "Drive to reef to score preloaded coral"),
        drivetrain.getPathCommand(ppPaths.get(0))
        
        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
