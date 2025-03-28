
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HID;

/**
 * A command that logs a string when initialized.
 */
public class AlignToReef extends InstantCommand
{
  private CommandSwerveDrivetrain m_swerve;
  private HID                     m_hid;

  /**
   * Creates a new a AlignToReef command
   * 
   * @param elevator
   *          elevator subsystem
   * @param manipulator
   *          manipulator subsystem
   * @param hid
   *          hid subsystem
   */
  public AlignToReef(CommandSwerveDrivetrain swerve, HID hid)
  {
    m_swerve = swerve;
    m_hid = hid;
  }

  @Override
  public void initialize( )
  {
    m_swerve.getAlignToReefCommand( ).schedule( );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return true;
  }
}
