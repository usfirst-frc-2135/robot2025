
package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CRConsts;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a coral to the reef
 */
public class ScoreCoralLevel extends SequentialCommandGroup
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
  public ScoreCoralLevel(Elevator elevator, Manipulator manipulator, LED led, HID hid, String level)
  {
    setName("ScoreCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(),"Move Manipulator To safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALHOLD, manipulator:: getMNSafePosition),


        new LogCommand(getName(), "Move Elevator to Position"),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralL4), //level 4

        new LogCommand(getName(), "Start rollers & Deploy Manipulator rotary"),
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.CORALEXPEL, manipulator::getCurrentPosition), //level 4

        // switch (level) {
        //   default :
        //       DataLogManager.log(String.format("%s: Claw mode is invalid: %s", getSubsystem( ), level));
        //   case "CORAL1" :
        //     new LogCommand(getName(), "Move Elevator to Position");
        //     elevator.getMoveToPositionCommand(elevator::getHeightCoralL4); //level 4
        //     new LogCommand(getName(), "Start rollers & Deploy Manipulator rotary");
        //     manipulator.getMoveToPositionCommand(CRConsts.ClawMode.CORALEXPEL, manipulator::getCurrentPosition);
        //     break;
        //   case "CORAL2" :
        //     new LogCommand(getName(), "Move Elevator to Position");
        //     elevator.getMoveToPositionCommand(elevator::getHeightCoralL4); //level 4
        //     new LogCommand(getName(), "Start rollers & Deploy Manipulator rotary");
        //     manipulator.getMoveToPositionCommand(CRConsts.ClawMode.CORALEXPEL, manipulator::getCurrentPosition);
        //     break;
        // }

        new LogCommand(getName(), "Wait for Coral"),
        // new WaitUntilCommand(manipulator::isCoralDetected), // checks if coral is expelled 
      
        new LogCommand(getName(), "Stop rollers"),
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.STOP, manipulator::getManipulatorRetracted)
        // elevator.getMoveToPositionCommand(elevator::getHeightStowed), // stowed
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
