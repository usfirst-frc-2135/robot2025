package frc.robot.commands;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VIConsts;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Swerve drive under PID control to a goal pose
 */
public class SwervePIDVisionTarget extends Command
{
  // Constants
  private static final LinearVelocity       kMaxSpeed                 = MetersPerSecond.of(3.5);     // Cap max applied velocity to 3.5 mps in either direction
  private static final AngularVelocity      kMaxAngularRate           = RotationsPerSecond.of(1.0); // Max 1.0 rot per second
  private static final AngularVelocity      kAngularVelocityTolerance = RotationsPerSecond.of(0.01);;
  private static final LinearVelocity       kSpeedTolerance           = InchesPerSecond.of(2.0);    // Was 0.25 inches per second which is extremely small

  // Main objects
  private CommandSwerveDrivetrain           m_swerve;
  private Vision                            m_vision;

  // Debouncer debouncer
  private static final Time                 kEndDebounce              = Seconds.of(0.04);
  private final Debouncer                   endDebouncer              =
      new Debouncer(kEndDebounce.in(Seconds), Debouncer.DebounceType.kBoth);
  private final BooleanPublisher            endConditionLogger        =
      ntInst.getTable("swerve").getBooleanTopic("PIDEndCondition").publish( );
  private boolean                           endCondition              = false;

  // Network tables entries
  private static final NetworkTableInstance ntInst                    = NetworkTableInstance.getDefault( );

  private static final NetworkTable         robotTable                = ntInst.getTable(Constants.kRobotString);
  private static final IntegerSubscriber    reefBranch                =
      robotTable.getIntegerTopic(VIConsts.kReefBranchString).subscribe((0));
  private static int                        m_selectedBranch;
  private static LinearVelocity             m_speed;
  private static AngularVelocity            m_rps;

  /**
   * Swerve drive under PID control to a goal pose
   */
  public SwervePIDVisionTarget(CommandSwerveDrivetrain swerve, Vision vision)
  {
    // TODO: The following line DOES NOT work. This creates a LOCAL variable called m_swerve which will
    //        No longer be available once this method hits the closing bracket (after the setName below on line 99)
    //        I commented it out and added the correct line of code needed.
    // CommandSwerveDrivetrain m_swerve = swerve;
    m_swerve = swerve;
    m_vision = vision;
    addRequirements(swerve);
    setName("SwervePIDVisionTarget");
  }

  /**
   * Command factory
   */
  public static Command generateCommand(CommandSwerveDrivetrain swerve, Vision vision, Time timeout)
  {
    return new SwervePIDVisionTarget(swerve, vision).withTimeout(timeout);
  }

  @Override
  public void initialize( )
  {
    // TODO: When we use the IntegerSubscriber type from network tables, we need to use reefBranch.get() below
    // TODO: We use a class member variable (m_selectedBranch), in case someone hits a controller button that tries to change the reefBranch during this command
    m_selectedBranch = (int) reefBranch.get( );
    DataLogManager.log(String.format("%s: initial reefBranch: %s ", getName( ), m_selectedBranch));
  }

  @Override
  public void execute( )
  {
    // TODO: Use the member variable holding the reefBranch (left or right)
    // TODO: Call the aimProportional and rangeProportional methods in the vision calls and pass in the left/right (or center) variable
    // TODO:    these should return the translation and angular velocit speeds that get put into the setControl
    // TODO: Comment out the swerve drive (but KEEP) setControl call until we can update it with the correct one

    // TODO: declare a LinearVelocity variable "speed" and set it to the return value from ...rangeProportional which returns a speed
    m_speed = m_vision.rangeProportional(m_selectedBranch, kMaxSpeed);

    // TODO: declare an AngularVelocity variable "rps" and set it to the return value from ...aimProportional which returns rotations per second
    m_rps = m_vision.aimProportional(m_selectedBranch, kMaxAngularRate);

    // TODO: We need a new m_swerve method that will take the linear speed (in chassis X direction) and rotations per second (like Apollo's code)
    m_swerve.setControl(new SwerveRequest.RobotCentric( ).withVelocityX(m_speed).withRotationalRate(m_rps));
  }

  @Override
  public void end(boolean interrupted)
  {
    // TODO: Update this logging call to make sense for our use - display the left/right, and end speeds
    // TODO: print what tx and ty are
    DataLogManager.log(String.format("%s: interrupted end conditions - speeds: %s rotation: %s", getName( ), m_speed, m_rps));
  }

  @Override
  public boolean isFinished( )
  {
    // TODO: Remove all of the pose references, keep a log message that prints only when the command is done
    // TODO: Our end condition will be the vision check plus the drive speed checks below

    boolean curSpeed = m_speed.lt(kSpeedTolerance);  // TODO: Does this handle negative speeds?

    boolean curRotation = m_rps.lt(kAngularVelocityTolerance);

    // TODO: Once working, comment out this log message because it spams the console
    DataLogManager.log(
        String.format("%s: end conditions for SwervePIDVisionTarget speed: %s rotation: %s", getName( ), curSpeed, curRotation));

    endCondition = endDebouncer.calculate(curSpeed);

    endConditionLogger.accept(endCondition);

    // TODO: Once working, comment out this log message because it spams the console
    DataLogManager.log(String.format("End condition: ", endCondition));

    return endCondition;
  }
}
