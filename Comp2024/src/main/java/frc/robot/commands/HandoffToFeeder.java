
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FDConsts;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Command to handoff from intake to feeder
 */
public class HandoffToFeeder extends SequentialCommandGroup
{
  /**
   * Group command to handoff from intake to feeder
   * 
   * @param intake
   *          intake subsystem
   * @param feeder
   *          feeder subsystem
   */
  public HandoffToFeeder(Intake intake, Feeder feeder, LED led)
  {
    setName("HandoffToFeeder");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Align Feeder and Intake"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.HANDOFF, feeder::getFeederHandoff),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getIntakeHandoff),

        new WaitCommand(0.25),

        new LogCommand(getName(), "Transfer Note"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.HOLD, feeder::getCurrentPosition),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.HANDOFF, intake::getCurrentPosition),
        
        new WaitCommand(0.1),

        new LogCommand(getName(), "Stop rollers"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.STOP, feeder::getCurrentPosition),

        new WaitCommand(0.2),

        new LogCommand(getName(), "Ensure Intake releases Note"),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getIntakeRetracted),
        led.getLEDCommand(COLOR.OFF, ANIMATION.CLEARALL),

        new LogCommand(getName(), "Align Feeder to Amp"),
        feeder.getMoveToPositionCommand(FDConsts.FDRollerMode.STOP, feeder::getFeederAmp)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
