
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CRConsts;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a algae to the reef
 */
public class AcquireAlgae extends SequentialCommandGroup
{
  /**
   * Group command to use the subsystems to acquire algae from the reef level 23
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
  public AcquireAlgae(Elevator elevator, Manipulator manipulator, LED led, HID hid)
  {
    setName("AcquireAlgae");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Move Elevator to Position"),
        elevator.getMoveToPositionCommand(elevator::getHeightAlgaeL23), //level 2/3

        new LogCommand(getName(), "Start rollers & Deploy Manipulator rotary"),
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.ALGAEACQUIRE, manipulator::getManipulatorAlgae23), //level 2/3
        
        new LogCommand(getName(), "Wait for Algae"),
        // new WaitUntilCommand(manipulator::isAlgaeDetected), // checks if algae is acquired 
      
        new LogCommand(getName(), "Stop rollers"),
        hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity)
        
        // elevator.getMoveToPositionCommand(elevator::getHeightStowed), // stowed
        // manipulator.getMoveToPositionCommand(CRConsts.ClawMode.STOP, manipulator::getManipulatorRetracted)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
