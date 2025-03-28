
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Manipulator;

/**
 * Command to acquire a coral from coral station
 */
public class AcquireCoral extends SequentialCommandGroup
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
   */
  public AcquireCoral(Elevator elevator, Manipulator manipulator, HID hid)
  {
    setName("AcquireCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(),"Move Manipulator to safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleSafeState),

        new LogCommand(getName(), "Move Elevator to coral station height"),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralStation),

        new LogCommand(getName(), "Start coral rollers to acquire & hold Manipulator in coral station position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALACQUIRE, manipulator::getAngleCoralStation),

        new LogCommand(getName(), "Wait for coral to be acquired"),
        new WaitUntilCommand(manipulator::isCoralDetected),
      
        new LogCommand(getName(), "Stop coral rollers move to safe state for next Elevator raise"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState),
        hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity)     

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
