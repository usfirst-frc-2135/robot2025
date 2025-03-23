package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Manipulator;

public class ExpelCoral extends SequentialCommandGroup
{

  public ExpelCoral(Elevator elevator, Manipulator manipulator, HID hid)
  {
    setName("ExpelCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName( ), "Start coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALEXPEL, manipulator::getCurrentAngle),

        new LogCommand(getName( ), "Wait for coral to expel"), 
        new WaitUntilCommand(manipulator::isCoralExpelled),
        new WaitCommand(0.2),

        new LogCommand(getName( ), "Stop coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState),

        new LogCommand(getName( ), "Move Elevator only to L2 height in case coral drops into robot"),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralL2)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }

}
