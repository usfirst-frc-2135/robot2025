
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FDConsts;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;

/**
 * Command to prepare for a climb
 */
public class PrepareToClimb extends SequentialCommandGroup
{
  /**
   * Group command to prepare robot for a climb
   * 
   * @param climber
   *          climber subsystem
   * @param feeder
   *          feeder subsystem
   */
  public PrepareToClimb(Climber climber, Feeder feeder)
  {
    setName("PrepareToClimb");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new ParallelCommandGroup( 
          feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.STOP, feeder::getFeederAmp),
          climber.getMoveToPositionCommand(climber::getClimberFullyExtended)
        )

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
