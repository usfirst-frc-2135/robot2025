
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Command to expel a note to the floor
 */
public class ExpelNote extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to expel a note to the floor
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public ExpelNote(Intake intake, LED led)
  {
    setName("ExpelNote");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Stop rollers & Deploy intake rotary"),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getIntakeDeployed),

        new LogCommand(getName(), "Expel rollers & Hold intake rotary in same position"),        
        intake.getMoveToPositionCommand(INConsts.INRollerMode.EXPEL, intake::getCurrentPosition),

        new LogCommand(getName(), "Wait for note to release"),
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers & Hold intake rotary in same position"),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getCurrentPosition),
        led.getLEDCommand(COLOR.OFF, ANIMATION.CLEARALL)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
