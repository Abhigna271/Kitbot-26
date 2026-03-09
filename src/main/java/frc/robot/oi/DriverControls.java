package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getMovement();

  public double getRotation();

  public Trigger intake();

  public Trigger rev();

  public Trigger shoot();
}
