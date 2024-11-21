
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

/**
 * Command to acquire a note from floor
 */
public class AcquireNote extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to acquire a note from the floor
   * 
   * @param intake
   *          intake subsystem
   * @param led
   *          led subsystem
   */
  public AcquireNote(Intake intake, LED led, HID hid)
  {
    setName("AcquireNote");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Start rollers & Deploy intake rotary"),
        led.getLEDCommand(COLOR.YELLOW, ANIMATION.CLEARALL),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.ACQUIRE, intake::getIntakeDeployed),

        new LogCommand(getName(), "Wait for note"),
        new WaitUntilCommand(intake::isNoteDetected),

        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
       
        hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Constants.kRumbleIntensity),
        hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Constants.kRumbleIntensity),
        
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
