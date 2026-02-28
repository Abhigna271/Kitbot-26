package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    public double voltage;
    public double current;
    public double output;
  }

  public void updateInputs(ShooterInputs inputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double supplyLimit);

  public void setPIDFF(double kP, double kI, double kD, double kS);
}
