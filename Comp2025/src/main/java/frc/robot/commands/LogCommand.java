
package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * A command that logs a string when initialized.
 */
public class LogCommand extends InstantCommand
{
  private String m_prefix;
  private String m_msg;

  /**
   * Creates a new a LogCommand.
   *
   * @param prefix
   *          the prefix before the message
   * @param message
   *          the message to log and print
   */
  public LogCommand(String prefix, String msg)
  {
    m_prefix = prefix;
    m_msg = msg;
  }

  @Override
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: %s", m_prefix, m_msg));
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return true;
  }
}
