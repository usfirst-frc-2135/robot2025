
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
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
   * @param led
   *          led subsystem
   */
  public AcquireCoral(Elevator elevator, Manipulator manipulator, LED led, HID hid)
  {
    setName("AcquireCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(),"Move Manipulator to safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN,manipulator::getAngleSafeState),

        new LogCommand(getName(), "Move Elevator to coral station height"),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralLStation),

        new LogCommand(getName(), "Start coral rollers & move Manipulator to coral station position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALACQUIRE, manipulator::getAngleCoralStation), // get coral from coral station

        new LogCommand(getName(), "Wait for coral"),
        new WaitUntilCommand(manipulator::isCoralDetected), // checks if coral is acquired 
      
        new LogCommand(getName(), "Stop coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getCurrentAngle),
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
