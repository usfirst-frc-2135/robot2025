
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
      case 4 : // POV 0
        return LevelSelector.NET;
      default :
      case 2 : // POV 180
        return LevelSelector.PROCESSOR;
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
        new LogCommand(getName(),"Move Manipulator to safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator:: getAngleAlgae23),

        new LogCommand(getName(), "Move Elevator to net height & move Manipulator to net position"),
        new SelectCommand<>( 
        // Maps selector values to commands 
        Map.ofEntries( 
            Map.entry(LevelSelector.NET, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeNet)), 
            Map.entry(LevelSelector.PROCESSOR, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeProcessor))), 
        this::selectLevel), 
 
      new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
      new SelectCommand<>( 
        // Maps selector values to commands 
        Map.ofEntries( 
            Map.entry(LevelSelector.NET, manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator::getAngleAlgaeNet)), 
            Map.entry(LevelSelector.PROCESSOR, manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator::getAngleCoralL2))),  
        this::selectLevel), 

      new LogCommand(getName(), "Start Claw Rollers"), 
      new SelectCommand<>( 
        // Maps selector values to commands 
        Map.ofEntries( 
            Map.entry(LevelSelector.NET, manipulator.getMoveToPositionCommand(ClawMode.ALGAESHOOT, manipulator::getCurrentAngle)), 
            Map.entry(LevelSelector.PROCESSOR, manipulator.getMoveToPositionCommand(ClawMode.ALGAEEXPEL, manipulator::getCurrentAngle))),  
        this::selectLevel), 
        
        new LogCommand(getName(), "Wait for algae"),
        new WaitCommand(Seconds.of(1.0)),
        // new WaitUntilCommand(manipulator::isAlgaeDetected), // checks if algae is expelled 
      
        new LogCommand(getName(), "Stop algae rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState),
        
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
