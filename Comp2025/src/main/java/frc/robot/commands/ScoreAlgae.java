
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.ELConsts.ReefLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a algae to the net
 */
public class ScoreAlgae extends SequentialCommandGroup
{

  private ReefLevel selectLevel( )
  {
    NetworkTable table = NetworkTableInstance.getDefault( ).getTable(Constants.kRobotString);
    int level = (int) table.getIntegerTopic(ELConsts.kReefLevelString).subscribe(0).get( );

    switch (level)
    {
      default : // Algae Processor
      case 1 :
      case 2 :
        return ReefLevel.ONE;
      case 3 : // Algae Net
      case 4 :
        return ReefLevel.TWO;
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
  public ScoreAlgae(Elevator elevator, Manipulator manipulator, HID hid)
  {
    setName("ScoreAlgae");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(),"Manipulator already at safe position - carrying algae"),
        // manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator:: getAngleAlgae23),

        new LogCommand(getName(), "Move Elevator to processor/net height"),
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(ReefLevel.ONE, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeProcessor)), 
                Map.entry(ReefLevel.TWO, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeNet))
              ), 
              this::selectLevel), 
 
        new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(ReefLevel.ONE, manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator::getAngleAlgaeProcessor)), 
                Map.entry(ReefLevel.TWO, manipulator.getMoveToPositionCommand(ClawMode.ALGAEMAINTAIN, manipulator::getAngleAlgaeNet))
              ),  
              this::selectLevel), 

        new LogCommand(getName(), "Start Claw Rollers"), 
        new SelectCommand<>( 
          // Maps selector values to commands 
          Map.ofEntries( 
                Map.entry(ReefLevel.ONE, manipulator.getMoveToPositionCommand(ClawMode.ALGAEEXPEL, manipulator::getCurrentAngle)), 
                Map.entry(ReefLevel.TWO, manipulator.getMoveToPositionCommand(ClawMode.ALGAESHOOT, manipulator::getCurrentAngle))
              ),  
              this::selectLevel), 
        
        new LogCommand(getName(), "Wait for algae"),
        new WaitCommand(Seconds.of(0.5)),
        // new WaitUntilCommand(manipulator::isAlgaeDetected), // checks if algae is expelled 
      
        new LogCommand(getName(), "Stop algae rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState), // Manipulator Safe State
        
        new LogCommand(getName(), "Move Elevator to stowed height"),
        elevator.getMoveToPositionCommand(elevator::getHeightStowed)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
