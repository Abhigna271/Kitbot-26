package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsPS5 implements DriverControls {
  private CommandPS5Controller m_controller;

  public DriverControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
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
    return m_controller.L2();
  }

  @Override
  public Trigger rev() {
    return m_controller.R2();
  }

  @Override
  public Trigger shoot() {
    return m_controller.R1();
  }
}
