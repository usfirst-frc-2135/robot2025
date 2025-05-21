package frc.robot.autos;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CRConsts.ClawMode;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HID;
import frc.robot.subsystems.Manipulator;

/**
 * Auto command that scores the preloaded coral and then two more
 */
public class AutoPreloadCoral2 extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Drive to a branch
   * 2 - Score a preloaded coral
   * 3 - Drive to coral station
   * 4 - Acquire a second coral
   * 5 - Drive to a branch
   * 6 - Score the second coral
   * 7 - Drive to coral station
   * 8 - Acquire a third coral
   * 9 - Drive to a branch
   * 10 - Score the third coral
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoPreloadCoral2(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
      Manipulator manipulator, HID hid, Supplier<Command> getReefLevelCommand)
  {
    setName("AutoPreloadCoral2");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Select scoring level"),
        getReefLevelCommand.get(),

        // Drive to branch

        new LogCommand(getName(), "PRELOAD: Drive to branch and score preload coral"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppPaths.get(0)),
          new SequentialCommandGroup(
            new LogCommand(getName(),"PRELOAD: Move Manipulator to safe position"),
            manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator:: getAngleSafeState),
            // Delay some time during the path before pre-raising the elevator
            new WaitCommand(0.7),
            new LogCommand(getName(),"PRELOAD: Move Elevator to L4 height early to save time"),
            elevator.getMoveToPositionCommand(elevator::getHeightCoralL4)
          )
        ),

        // Expel the Preload Coral

        new LogCommand(getName( ), "PRELOAD: Move Manipulator to L4 scoring state"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL4),
        new LogCommand(getName( ), "PRELOAD: Start coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALEXPEL, manipulator::getCurrentAngle),
        new LogCommand(getName( ), "PRELOAD: Wait for coral to expel"), 
        new WaitUntilCommand(manipulator::isCoralExpelled).withTimeout(0.3), // Coral takes about 0.250 to pass through sensor plus some extra margin
        new WaitCommand(0.050),                                                 // Wait just a little longer to ensure it completely exits
        new LogCommand(getName( ), "PRELOAD: Stop coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState),

        // First Coral Station

        new LogCommand(getName(), "CORAL1: Drive to coral station and acquire second coral"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppPaths.get(1)),
          new SequentialCommandGroup(
            new LogCommand(getName(), "CORAL1: Move Elevator to coral station height"),
            elevator.getMoveToPositionCommand(elevator::getHeightCoralStation),
            new LogCommand(getName(), "CORAL1: Start coral rollers to acquire & hold Manipulator in coral station position"),
            manipulator.getMoveToPositionCommand(ClawMode.CORALACQUIRE, manipulator::getAngleCoralStation)
          )
        ), 
    
        new LogCommand(getName(), "CORAL1: Wait for coral to be acquired"),
        new WaitUntilCommand(manipulator::isCoralDetected),
        // new WaitUntilCommand(manipulator::isCoralDetected).withTimeout(0.3),    // Temporary for debugging
        new LogCommand(getName(), "CORAL1: Stop coral rollers move to safe state for next Elevator raise"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator:: getCurrentAngle),

        // Drive to branch

        new LogCommand(getName(), "CORAL1: Drive to branch and score second coral"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppPaths.get(2)),
          new SequentialCommandGroup(
            new LogCommand(getName(),"CORAL1: Move Manipulator to safe position"),
            manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator:: getAngleSafeState), 
            // Delay some time during the path before pre-raising the elevator
            new WaitCommand(0.8),
            new LogCommand(getName(),"CORAL1: Move Elevator to L4 height early to save time"),
            elevator.getMoveToPositionCommand(elevator::getHeightCoralL4)
          )
        ),

        // Expel the First Coral

        new LogCommand(getName( ), "CORAL1: Move Manipulator to L4 scoring state"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL4),
        new LogCommand(getName( ), "CORAL1: Start coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALEXPEL, manipulator::getCurrentAngle),
        new LogCommand(getName( ), "CORAL1: Wait for coral to expel"), 
        new WaitUntilCommand(manipulator::isCoralExpelled).withTimeout(0.3), // Coral takes about 0.250 to pass through sensor plus some extra margin
        new WaitCommand(0.050),                                                 // Wait just a little longer to ensure it completely exits
        new LogCommand(getName( ), "CORAL1: Stop coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState),

        // Second Coral Station

        new LogCommand(getName(), "CORAL2: Drive to coral station and acquire third coral"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppPaths.get(3)),
          new SequentialCommandGroup(
            new LogCommand(getName(), "CORAL2: Move Elevator to coral station height"),
            elevator.getMoveToPositionCommand(elevator::getHeightCoralStation),
            new LogCommand(getName(), "CORAL2: Start coral rollers to acquire & hold Manipulator in coral station position"),
            manipulator.getMoveToPositionCommand(ClawMode.CORALACQUIRE, manipulator::getAngleCoralStation)
          )
        ), 
    
        new LogCommand(getName(), "CORAL2: Wait for coral to be acquired"),
        new WaitUntilCommand(manipulator::isCoralDetected),
        // new WaitUntilCommand(manipulator::isCoralDetected).withTimeout(0.3),    // Temporary for debugging
        new LogCommand(getName(), "CORAL2: Stop coral rollers move to safe state for next Elevator raise"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator:: getCurrentAngle),

        // Drive to branch

        new LogCommand(getName(), "CORAL2: Drive to branch and score third coral"),   
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppPaths.get(4)),
          new SequentialCommandGroup(
            new LogCommand(getName(),"CORAL2: Move Manipulator to safe position"),
            manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator:: getAngleSafeState), 
            // Delay some time during the path before pre-raising the elevator
            new WaitCommand(0.8),
            new LogCommand(getName(),"CORAL2: Move Elevator to L4 height early to save time"),
            elevator.getMoveToPositionCommand(elevator::getHeightCoralL4)
          )
        ),

        // Expel the Second Coral

        new LogCommand(getName( ), "CORAL2: Move Manipulator to L4 scoring state"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, manipulator::getAngleCoralL4),
        new LogCommand(getName( ), "CORAL2: Start coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.CORALEXPEL, manipulator::getCurrentAngle),
        new LogCommand(getName( ), "CORAL2: Wait for coral to expel"), 
        new WaitUntilCommand(manipulator::isCoralExpelled).withTimeout(0.3), // Coral takes about 0.250 to pass through sensor plus some extra margin
        new WaitCommand(0.050),                                                 // Wait just a little longer to ensure it completely exits
        new LogCommand(getName( ), "CORAL2: Stop coral rollers"),
        manipulator.getMoveToPositionCommand(ClawMode.STOP, manipulator::getAngleSafeState),

        // Return elevator down

        new LogCommand(getName(), "CORAL2: Move Elevator to coral station height"),
        elevator.getMoveToPositionCommand(elevator::getHeightCoralStation)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
