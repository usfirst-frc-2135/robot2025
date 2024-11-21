
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Command to retract the intake
 */
public class RetractIntake extends SequentialCommandGroup
{
  /**
   * Group command to move the intake to retracted position
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public RetractIntake(Intake intake, LED led, HID hid)
  {
    setName("RetractIntake");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
        new ConditionalCommand(
          led.getLEDCommand(COLOR.BLUE, ANIMATION.CLEARALL),
          led.getLEDCommand(COLOR.OFF, ANIMATION.CLEARALL),
          intake::isNoteDetected),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP, intake::getIntakeRetracted)

        // @formatter:on
    );

  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
