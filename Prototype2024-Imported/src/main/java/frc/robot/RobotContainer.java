package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.Servo;
// import frc.robot.commands.ActuatorIn;
// import frc.robot.commands.ActuatorOut;

public class RobotContainer {
    private final Servo m_actuator = new Servo(0);

    private final SubsystemBase actuatorSS = new SubsystemBase() {
    };

    // private SendableChooser<ActuatorChooser> m_actuatorChooser =
    // newSendableChooser<>();

    // public enum ActuatorChooser {
    // ActuatorOut,
    // ActuatorIn
    // }

    private void AddDashboardWidgets() {

        SmartDashboard.putData("IN (hold)", Commands.startEnd(() -> m_actuator.set(0.75), () -> m_actuator.set(0.25),
                actuatorSS));
        SmartDashboard.putData("OUT (hold)", Commands.startEnd(() -> m_actuator.set(0.25),
                () -> m_actuator.set(0.75),
                actuatorSS));

        // SmartDashboard.putData("ActuatorIn", new ActuatorIn(m_actuator));
        // SmartDashboard.putData("ActuatorOut", new ActuatorOut(m_actuator));
        // SmartDashboard.putNumber("test", 1);

        // m_actuatorChooser.onChange(this::updateActutorChooserCallback);

    }

    public RobotContainer() {
        AddDashboardWidgets(); // Add some dashboard widgets for commands

    }

}
