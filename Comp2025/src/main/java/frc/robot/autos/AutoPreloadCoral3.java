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
public class AutoPreloadCoral3 extends SequentialCommandGroup
{

  // Drive to score

  private Command driveToScore(String phase, PathPlannerPath path, CommandSwerveDrivetrain drive, Elevator elev, Manipulator man,
      double delayUntilElevatorUp)
  {
    return new SequentialCommandGroup(                                                      //
        new LogCommand(getName( ), phase + ": Drive to branch and score coral"),            //
        new ParallelCommandGroup(                                                           //
            drive.getPathCommand(path),                                                     //  ~1.6 sec
            new SequentialCommandGroup(                                                     //
                man.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, man::getAngleSafeState), // ~275 msec
                // Delay some time during the path before pre-raising the elevator
                new WaitCommand(delayUntilElevatorUp),                                      //
                elev.getMoveToPositionCommand(elev::getHeightCoralL4))                      // ~900 msec
        )                                                                                   //
    );
  }

  // Expel the Preload Coral

  private Command expelCoral(String phase, Elevator elev, Manipulator man)
  {
    return new SequentialCommandGroup(                                                      //
        new LogCommand(getName( ), phase + ": Expel coral and wait until complete"),        //
        man.getMoveToPositionCommand(ClawMode.CORALMAINTAIN, man::getAngleCoralL4),         //
        man.getMoveToPositionCommand(ClawMode.CORALEXPEL, man::getCurrentAngle),            //
        new WaitUntilCommand(man::isCoralExpelled).withTimeout(0.3),                // Coral takes about 0.250 to pass through sensor plus some extra margin
        new WaitCommand(0.050),                                                     // Wait just a little longer to ensure it completely exits
        man.getMoveToPositionCommand(ClawMode.STOP, man::getAngleSafeState)                 //
    );
  }

  // Drive to score

  private Command driveToAcquire(String phase, PathPlannerPath path, CommandSwerveDrivetrain drive, Elevator elev,
      Manipulator man)
  {
    return new SequentialCommandGroup(                                                      //
        new LogCommand(getName( ), phase + ": Drive to coral station and acquire"),         //
        new ParallelCommandGroup(                                                           //
            drive.getPathCommand(path),                                                     //
            new SequentialCommandGroup(                                                     //
                elev.getMoveToPositionCommand(elev::getHeightCoralStation),                 //
                man.getMoveToPositionCommand(ClawMode.CORALACQUIRE, man::getAngleCoralStation)) //
        )                                                                                   //
    );
  }

  private Command acquireCoral(String phase, Manipulator man)
  {
    return new SequentialCommandGroup(                                                      //

        new LogCommand(getName( ), phase + ": Wait for coral to be acquired and stop rollers"), //
        new WaitUntilCommand(man::isCoralDetected),
        // new WaitUntilCommand(manipulator::isCoralDetected).withTimeout(0.3),             // Temporary for debugging
        man.getMoveToPositionCommand(ClawMode.STOP, man::getCurrentAngle));
  }

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
   * 11 - Drive to coral station
   * 12 - Drive to a branch
   * 13 - Score the fourth coral
   * 
   * @param ppPaths
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoPreloadCoral3(List<PathPlannerPath> ppPaths, CommandSwerveDrivetrain drivetrain, Elevator elevator,
      Manipulator manipulator, HID hid, Supplier<Command> getReefLevelCommand)
  {
    setName("AutoPreloadCoral3");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Select scoring level"),
        getReefLevelCommand.get(),

        // Score preloaded coral

        driveToScore("PRELOAD", ppPaths.get(0), drivetrain, elevator, manipulator, 0.470),
        expelCoral("PRELOAD", elevator, manipulator),

        // First Coral Station

        
        driveToAcquire("CORAL1", ppPaths.get(1), drivetrain, elevator, manipulator),
        acquireCoral("CORAL1", manipulator),
        driveToScore("CORAL1", ppPaths.get(2), drivetrain, elevator, manipulator, 0.600),
        expelCoral("CORAL1", elevator, manipulator),

        // Second Coral Station

        driveToAcquire("CORAL2", ppPaths.get(3), drivetrain, elevator, manipulator),
        acquireCoral("CORAL2", manipulator),
        driveToScore("CORAL2", ppPaths.get(4), drivetrain, elevator, manipulator,0.600),
        expelCoral("CORAL2", elevator, manipulator),
    
        // Third Coral Station

        driveToAcquire("CORAL3", ppPaths.get(5), drivetrain, elevator, manipulator),
        acquireCoral("CORAL3", manipulator),
        driveToScore("CORAL3", ppPaths.get(6), drivetrain, elevator, manipulator, 0.600),
        expelCoral("CORAL3", elevator, manipulator)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
