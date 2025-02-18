
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CRConsts;
import frc.robot.Constants.LEDConsts.ANIMATION;
import frc.robot.Constants.LEDConsts.COLOR;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a coral to the reef
 */
public class AquireAlgae extends SequentialCommandGroup
{
  /**
   * Group command to use the subsystems to acquire a coral from the coral station
   * 
   * @param elevator
   *          elevator subsystem
   * @param manipulator
   *          manipulator subsystem
   * @param hid
   *          hid subsystem
   * @param led
   *          led subsystem
   */
  public AquireAlgae(Elevator elevator, Manipulator manipulator, LED led, HID hid)
  {
    setName("AquireAlgae");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new LogCommand(getName(), "Drive to face on reef"),

        new LogCommand(getName(), "Move Elevator to Position"),
        elevator.getMoveToPositionCommand(elevator::getHeightAlgaeL23), //level 2/3
        // elevator.getMoveToPositionCommand(elevator::getHeightAlgaeL34), //level 3/4
        // elevator.getMoveToPositionCommand(elevator::getHeightAlgaeNet),// Net

        new LogCommand(getName(), "Start rollers & Deploy Manipulator rotary"),
        // led.getLEDCommand(COLOR.YELLOW, ANIMATION.CLEARALL),// 
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.ALGAEACQUIRE, manipulator::getManipulatorAlgae23), //level 2/3
        // manipulator.getMoveToPositionCommand(CRConsts.ClawMode.ALGAEACQUIRE, manipulator::getManipulatorAlgae34),//level 3/4
        
        new LogCommand(getName(), "Wait for Coral"),
        // new WaitUntilCommand(manipulator::coralDetected), // checks if coral is detected 
      
        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
       
        hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.STOP, manipulator::getManipulatorAlgae23)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
