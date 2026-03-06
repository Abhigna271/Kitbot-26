package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerInputs {
    public double voltage;
    public double current;
    public double output;
  }

  public void updateInputs(KickerInputs inputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double supplyLimit);
}
