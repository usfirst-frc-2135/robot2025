
package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.Constants.ELConsts;
import frc.robot.Constants.ELConsts.ReefLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;

/**
 * Command to score a algae to the reef
 */
public class AcquireAlgae extends SequentialCommandGroup
{

  private ReefLevel selectLevel( )
  {
    NetworkTable table = NetworkTableInstance.getDefault( ).getTable(Constants.kRobotString);
    int level = (int) table.getIntegerTopic(ELConsts.kReefLevelString).subscribe(0).get( );

    switch (level)
    {
      case 2 :
      case 1 : // Algae Level 23 - POV 270
        return ReefLevel.ONE;
      default :
      case 3 : // Algae Level 34 - POV 90
      case 4 :
        return ReefLevel.THREE;
    }
  }

  /**
   * Group command to use the subsystems to acquire algae from the reef
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
  public AcquireAlgae(Elevator elevator, Manipulator manipulator, LED led, HID hid)
  {
    setName("AcquireAlgae");

    addCommands(
        // Add Commands here:

        // @formatter:off
        new LogCommand(getName(),"Move Manipulator to safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator:: getAngleSafeState),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralLStation),
        
      new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
      new SelectCommand<>( 
        // Maps selector values to commands 
        Map.ofEntries( 
            Map.entry(ReefLevel.ONE, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeL23)), 
            Map.entry(ReefLevel.THREE, elevator.getMoveToPositionCommand(elevator::getHeightAlgaeL34))),  
        this::selectLevel), 

        new LogCommand(getName(), "Move Manipulator to reef position based on the level"), 
      new SelectCommand<>( 
        // Maps selector values to commands 
        Map.ofEntries( 
            Map.entry(ReefLevel.ONE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleAlgae23)), 
            Map.entry(ReefLevel.THREE, manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleAlgae34))), 
        this::selectLevel),

        manipulator.getMoveToPositionCommand(ClawMode.ALGAEACQUIRE, manipulator::getCurrentAngle), //level 2/3
        
        new LogCommand(getName(), "Wait for algae"),
        new WaitCommand(Seconds.of(1.0)),
        // new WaitUntilCommand(manipulator::isAlgaeDetected), // checks if algae is acquired 
      
        new LogCommand(getName(), "Stop algae rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.ALGAEHOLD, manipulator::getCurrentAngle),
        
        hid.getHIDRumbleDriverCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        hid.getHIDRumbleOperatorCommand(Constants.kRumbleOn, Seconds.of(1.0), Constants.kRumbleIntensity),
        
        new LogCommand(getName(), "Move Elevator to safe height"),
        elevator.getMoveToPositionCommand(elevator::getHeightAlgaeL23), // stowed

        new LogCommand(getName(), "Move Manipulator to Algae safe position"),
        manipulator.getMoveToPositionCommand(ClawMode.ALGAEHOLD, manipulator::getCurrentAngle) // Manipulator Safe State with Algae
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
