
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FDConsts;
import frc.robot.subsystems.Elevator;

/**
 * Command to prepare for a climb
 */
public class RunElevator extends SequentialCommandGroup
{
  /**
   * Group command to prepare robot for a climb
   * 
   * @param elevator
   *          elevator subsystem
   * 
   */
  public RunElevator(Elevator elevator)
  {
    setName("RunElevator");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new ParallelCommandGroup( 
          Elevator.getMoveToPositionCommand(elevator::getElevatorCoralL1)
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
