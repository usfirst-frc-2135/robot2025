
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a coral to the reef
 */
public class ScoreCoralLevels extends SequentialCommandGroup
{
  /**
   * Group command to use the subsystems to score a coral in the reef level 4
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
  public ScoreCoralLevels(Elevator elevator, Manipulator manipulator, LED led, HID hid, String level, String reefLevel)
  {
    setName("ScoreCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(),"Move Manipulator to safe position"),
        manipulator.getMoveToPositionCommand(level),

        new LogCommand(getName(), "Move Elevator to Reef L4 height"),
        elevator.getMoveToPositionCommand(reefLevel), //level 4

        // new LogCommand(getName(), "Move Manipulator to reef L4 position"),
        // manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL4),
        

        new LogCommand(getName(), "Start coral rollers"),
        manipulator.getMoveToPositionCommand(level), //level 4

        new LogCommand(getName(), "Wait for coral to expel"),
        new WaitUntilCommand(manipulator::isCoralExpelled), // checks if coral is expelled 
      
        new LogCommand(getName(), "Stop coral rollers"),
        manipulator.getMoveToPositionCommand(level),

        new LogCommand(getName(), "Move Elevator to coral station height"),
        elevator.getMoveToPositionCommand(reefLevel) // coral station height
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
