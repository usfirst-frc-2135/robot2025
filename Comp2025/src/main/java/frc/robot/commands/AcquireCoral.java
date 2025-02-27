
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CRConsts;
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
        new LogCommand(getName(),"Move Manipulator To safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALHOLD,manipulator::getMNSafePosition),


        new LogCommand(getName(), "Move Elevator to Position"),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralLStation),
        // elevator.getMoveToPositionCommand(elevator::getHeightCoralL4),

        new LogCommand(getName(), "Start rollers & Deploy Manipulator rotary"),
        manipulator.getMoveToPositionCommand(CRConsts.ClawMode.CORALACQUIRE, manipulator::getCurrentPosition), // get coral from coral station

        new LogCommand(getName(), "Wait for Coral"),
        new WaitUntilCommand(manipulator::isCoralDetected), // checks if coral is acquired 
      
        new LogCommand(getName(), "Stop rollers"),
        hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity)
        
        // elevator.getMoveToPositionCommand(elevator::getHeightStowed), // stowed
        // manipulator.getMoveToPositionCommand(CRConsts.ClawMode.STOP, manipulator::getCurrentPosition)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
