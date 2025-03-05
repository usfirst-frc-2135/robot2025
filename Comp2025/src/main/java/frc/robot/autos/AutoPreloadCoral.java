
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Auto command that can be used for testing new sequences
 */
public class AutoPreloadCoral extends SequentialCommandGroup
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
    public AutoPreloadCoral(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
            Manipulator manipulator, LED led, HID hid)
    {
        setName("AutoPreloadCoral");

        addCommands(
                // Add Commands here:

                // @formatter:off

        new LogCommand(getName(), "Drive a test path"),
        drivetrain.getPathCommand(ppPaths.get(0)),
        drivetrain.getPathCommand(ppPaths.get(1)),
        drivetrain.getPathCommand(ppPaths.get(2))
        
        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
