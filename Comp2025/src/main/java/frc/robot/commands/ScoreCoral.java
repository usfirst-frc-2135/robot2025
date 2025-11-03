
package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.ELConsts.ReefLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a coral to the reef
 */
public class ScoreCoral extends SequentialCommandGroup
{

  private ReefLevel selectLevel( )
  {
    NetworkTable table = NetworkTableInstance.getDefault( ).getTable(Constants.kRobotString);
    int level = (int) table.getIntegerTopic(ELConsts.kReefLevelString).subscribe(0).get( );

    switch (level)
    {
      case 1 : // Coral Level 1
        return ReefLevel.ONE;
      case 2 : // Coral Level 2
        return ReefLevel.TWO;
      case 3 : // Coral Level 3
        return ReefLevel.THREE;
      default :
      case 4 : // Coral Level 4
        return ReefLevel.FOUR;
    }
  }

  /**
   * Group command to use the subsystems to score a coral in the reef level 4
   * 
   * @param elevator
   *          elevator subsystem
   * @param manipulator
   *          manipulator subsystem
   */
  public ScoreCoral(Elevator elevator, Manipulator manipulator)
  {
    setName("ScoreCoral");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(),"Move Manipulator to safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator:: getAngleSafeState),  

        new LogCommand(getName(), "Move Elevator to reef height based on the level"), 
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(ReefLevel.ONE, elevator.getMoveToPositionCommand(elevator::getHeightCoralL1)), 
                Map.entry(ReefLevel.TWO, elevator.getMoveToPositionCommand(elevator::getHeightCoralL2)), 
                Map.entry(ReefLevel.THREE, elevator.getMoveToPositionCommand(elevator::getHeightCoralL3)), 
                Map.entry(ReefLevel.FOUR, elevator.getMoveToPositionCommand(elevator::getHeightCoralL4))
              ), 
              this::selectLevel
          ), 

        new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(ReefLevel.ONE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL1)), 
                Map.entry(ReefLevel.TWO, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL2)), 
                Map.entry(ReefLevel.THREE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL3)), 
                Map.entry(ReefLevel.FOUR, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL4))
              ), 
              this::selectLevel)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
