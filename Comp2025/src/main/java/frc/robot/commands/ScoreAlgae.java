
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants.ELConsts.LevelSelector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a algae to the net
 */
public class ScoreAlgae extends SequentialCommandGroup
{

  private LevelSelector selectLevel( )
  {
    NetworkTable table = NetworkTableInstance.getDefault( ).getTable("robotContainer");
    int level = (int) table.getIntegerTopic("ReefLevel").subscribe(0).get( );

    switch (level)
    {
      default : // Algae Processor
      case 1 :
      case 2 :
        return LevelSelector.ONE;
      case 3 : // Algae Net
      case 4 :
        return LevelSelector.TWO;
    }
  }

  /**
   * Group command to use the subsystems to score a algae in the net
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
  public ScoreAlgae(Elevator elevator, Manipulator manipulator, LED led, HID hid)
  {
    setName("ScoreAlgae");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(),"Manipulator already at safe position - carrying algae"),
        // manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator:: getAngleAlgae23),

        new LogCommand(getName(), "Move Elevator to processor/net height & move Manipulator to processor/net position"),
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(LevelSelector.ONE, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeProcessor)), 
                Map.entry(LevelSelector.TWO, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeNet))
              ), 
              this::selectLevel), 
 
        new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(LevelSelector.ONE, manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator::getAngleAlgaeProcessor)), 
                Map.entry(LevelSelector.TWO, manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator::getAngleAlgaeNet))
              ),  
              this::selectLevel), 

        new LogCommand(getName(), "Start Claw Rollers"), 
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(LevelSelector.ONE, manipulator.getMoveToPositionCommand(ClawMode.ALGAEEXPEL, manipulator::getCurrentAngle)), 
                Map.entry(LevelSelector.TWO, manipulator.getMoveToPositionCommand(ClawMode.ALGAESHOOT, manipulator::getCurrentAngle))
              ),  
              this::selectLevel), 
        
        new LogCommand(getName(), "Wait for algae"),
        new WaitCommand(Seconds.of(1.0)),
        // new WaitUntilCommand(manipulator::isAlgaeDetected), // checks if algae is expelled 
      
        new LogCommand(getName(), "Stop algae rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState), // Manipulator Safe State
        
        new LogCommand(getName(), "Move Elevator to stowed height"),
        elevator.getMoveToPositionCommand(elevator::getHeightStowed) // stowed
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
