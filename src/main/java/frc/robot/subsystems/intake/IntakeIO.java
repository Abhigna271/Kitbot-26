package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double voltage;
    public double current;
    public double output;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double voltage);

  public void setCurrentLimits(double supplyLimit);

  public void setPIDFF(double kP, double kI, double kD, double kS);
}
