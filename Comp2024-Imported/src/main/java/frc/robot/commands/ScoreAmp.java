
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FDConsts;
import frc.robot.subsystems.Feeder;

/**
 * Command to score a note into amp
 */
public class ScoreAmp extends SequentialCommandGroup
{
  /**
   * Group command to use the feeder to score a note in the amp
   * 
   * @param feeder
   *          feeder subsystem
   */
  public ScoreAmp(Feeder feeder)
  {
    setName("ScoreAmp");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Score Note to Amp"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.SCORE, feeder::getCurrentPosition),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.STOP, feeder::getFeederHandoff)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
