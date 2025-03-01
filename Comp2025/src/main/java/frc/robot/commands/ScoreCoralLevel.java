
package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants.ELCommandSelector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a coral to the reef
 */
public class ScoreCoralLevel extends SequentialCommandGroup
{

  private ELCommandSelector select( )
  {
    return ELCommandSelector.ONE;
  }

  private ELCommandSelector selector( )
  {
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("robotContainer");

    IntegerSubscriber reefLevel = table.getIntegerTopic("ReefLevel").subscribe(0);
    int level = (int) reefLevel.get( );

    switch (level)
    {
      default :
      case 1 :
        return ELCommandSelector.ONE;
      case 2 :
        return ELCommandSelector.TWO;
      case 3 :
        return ELCommandSelector.THREE;
      case 4 :
        return ELCommandSelector.FOUR;
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
  public ScoreCoralLevel(Elevator elevator, Manipulator manipulator, LED led, HID hid)
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
            Map.entry(ELCommandSelector.ONE, elevator.getMoveToPositionCommand(elevator::getHeightCoralL1)), 
            Map.entry(ELCommandSelector.TWO, elevator.getMoveToPositionCommand(elevator::getHeightCoralL2)), 
            Map.entry(ELCommandSelector.THREE, elevator.getMoveToPositionCommand(elevator::getHeightCoralL3)), 
            Map.entry(ELCommandSelector.FOUR, elevator.getMoveToPositionCommand(elevator::getHeightCoralL4))), 
        this::selector), 
 
      new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
      new SelectCommand<>( 
        // Maps selector values to commands 
        Map.ofEntries( 
            Map.entry(ELCommandSelector.ONE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL1)), 
            Map.entry(ELCommandSelector.TWO, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL2)), 
            Map.entry(ELCommandSelector.THREE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL3)), 
            Map.entry(ELCommandSelector.FOUR, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL4))), 
        this::selector), 

        new LogCommand(getName(), "Start coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALEXPEL, manipulator::getCurrentAngle), //level 4
  
        new LogCommand(getName(), "Wait for coral to expel"),
        new WaitUntilCommand(manipulator::isCoralExpelled), // checks if coral is expelled 
      
        new LogCommand(getName(), "Stop coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState),
  
        new LogCommand(getName(), "Move Elevator to coral station height"),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralLStation) // coral station height
              
      // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
