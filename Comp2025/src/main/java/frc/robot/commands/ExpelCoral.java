package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class ExpelCoral extends SequentialCommandGroup
{
  /**
   * Group command to use the subsystems to expel a coral onto the reef
   * 
   * @param elevator
   *          elevator subsystem
   * @param manipulator
   *          manipulator subsystem
   */
  public ExpelCoral(Elevator elevator, Manipulator manipulator)
  {
    setName("ExpelCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName( ), "Start coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALEXPEL, manipulator::getCurrentAngle),

        new LogCommand(getName( ), "Wait for coral to expel"), 
        new WaitUntilCommand(manipulator::isCoralExpelled).withTimeout(0.5), // Coral takes about 0.250 to pass through sensor plus some extra margin
        new WaitCommand(0.25),                                                 // Wait just a little longer to ensure it completely exits

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
