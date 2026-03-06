package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXbox implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
  }

  @Override
  public double getMovement() {
    return m_controller.getLeftY();
  }

  @Override
  public double getRotation() {
    return m_controller.getRightX();
  }

  @Override
  public Trigger intake() {
    return m_controller.leftTrigger();
  }

  @Override
  public Trigger rev() {
    return m_controller.rightTrigger();
  }

  @Override
  public Trigger shoot() {
    return m_controller.rightBumper();
  }
}
