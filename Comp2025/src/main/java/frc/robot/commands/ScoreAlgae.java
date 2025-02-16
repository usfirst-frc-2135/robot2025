
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
public class ScoreAlgae extends SequentialCommandGroup
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
  public ScoreAlgae(Elevator elevator, Manipulator manipulator, LED led, HID hid)
  {
    setName("ScoreAlgae");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new LogCommand(getName(), "Drive to face on reef"),

        new LogCommand(getName(), "Move Elevator to Position"),
        elevator.getMoveToPositionCommand(elevator::getHeightAlgaeProcessor), 

        new LogCommand(getName(), "Start rollers & Deploy Manipulator rotary"),
        // led.getLEDCommand(COLOR.YELLOW, ANIMATION.CLEARALL), //TODO: change LED
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.CORALEXPEL, manipulator::getManipulatorAlgaeProcessor), 
        
        new LogCommand(getName(), "Wait for Coral"),
        // new WaitUntilCommand(manipulator::AlgaeDetected), // checks if coral is detected 
      
        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
       
        hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.STOP, manipulator::getManipulatorRetracted)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
