package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer
{
  // In a real robot, the servo actuator would be declared within a subsystem and not here
  private final Servo m_actuator = new Servo(0);

  private void AddDashboardWidgets( )
  {
    SmartDashboard.putData("Actuator_IN_Open", Commands.runOnce(( ) -> m_actuator.setSpeed(1.0)));
    SmartDashboard.putData("Actuator_OUT_Close", Commands.runOnce(( ) -> m_actuator.setSpeed(-1.0)));
  }

  public RobotContainer( )
  {
    // Change servo settings to work with this linear actuator per the docs linked on this web page:
    // https://andymark.com/products/linear-servo-actuators?_pos=1&_sid=800c6cc56&_ss=r
    // Max PWM pulse 2.0 ms, min 1.0 ms, with reasonable middle ranges
    m_actuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    AddDashboardWidgets( ); // Add dashboard widgets for commands
  }

}
