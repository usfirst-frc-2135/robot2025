
package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants.ELConsts.LevelSelector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a coral to the reef
 */
public class ScoreCoral extends SequentialCommandGroup
{

  private LevelSelector selectLevel( )
  {
    NetworkTable table = NetworkTableInstance.getDefault( ).getTable("robotContainer");
    int level = (int) table.getIntegerTopic("ReefLevel").subscribe(0).get( );

    switch (level)
    {
      case 1 : // Coral Level 1
        return LevelSelector.ONE;
      case 2 : // Coral Level 2
        return LevelSelector.TWO;
      case 3 : // Coral Level 3
        return LevelSelector.THREE;
      default :
      case 4 : // Coral Level 4
        return LevelSelector.FOUR;
    }
  }

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
  public ScoreCoral(Elevator elevator, Manipulator manipulator, LED led, HID hid)
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
                Map.entry(LevelSelector.ONE, elevator.getMoveToPositionCommand(elevator::getHeightCoralL1)), 
                Map.entry(LevelSelector.TWO, elevator.getMoveToPositionCommand(elevator::getHeightCoralL2)), 
                Map.entry(LevelSelector.THREE, elevator.getMoveToPositionCommand(elevator::getHeightCoralL3)), 
                Map.entry(LevelSelector.FOUR, elevator.getMoveToPositionCommand(elevator::getHeightCoralL4))
              ), 
              this::selectLevel
          ), 

        new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(LevelSelector.ONE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL1)), 
                Map.entry(LevelSelector.TWO, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL2)), 
                Map.entry(LevelSelector.THREE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL3)), 
                Map.entry(LevelSelector.FOUR, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL4))
              ), 
              this::selectLevel), 

        new LogCommand(getName(), "Start coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALEXPEL, manipulator::getCurrentAngle), //level 4

        new LogCommand(getName(), "Wait for coral to expel"),
        new WaitCommand(0.25),
        new WaitUntilCommand(manipulator::isCoralExpelled), // checks if coral is expelled 
      
        new LogCommand(getName(), "Stop coral rollers"), 
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState), // Manipulator Safe State 

        new LogCommand(getName(), "Move Elevator to stowed height"),
        elevator.getMoveToPositionCommand(elevator::getHeightStowed) // coral station height
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
