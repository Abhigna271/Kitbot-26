package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double velocityRPS;
    public double accelerationRPSSq;
    public double current;
    public double statorCurrent;
    public double voltage;
    public double temperature;
    public boolean motorIsConnected;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double volt);

  public void setCurrentLimits(double supplyLimit);
}
