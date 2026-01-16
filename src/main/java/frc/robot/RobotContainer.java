package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Controller
  private DriverControls m_controller;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureBindings();
  }

  public void configureSubsystems() {

  }

  public void configureCommands() {
  }

  public void configureControllers() {
    m_controller = new DriverControlsPS5(0);
  }

  public void configureBindings() {
    m_controller
        .intake()
        .onTrue(
            Commands.runOnce(
                (() -> {
                  if(RobotState.getInstance().getCurrAction() != RobotAction.kIntaking) {
                  RobotState.getInstance().updateRobotAction(RobotAction.kIntaking);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kTeleopDefault);
                  }
                })));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public String getSelectedAuto() {
    return m_autoChooser.getSendableChooser().getSelected();
  }
}
