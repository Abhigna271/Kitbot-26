package frc.robot.subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;
// IO is js the blueprint, showing all the necessities of an Intake file

public interface KickerIO {
  @AutoLog
  public static class KickerInputs {
    public double voltage;
    public double current;
    public double output;
  }

  public void updateInputs(KickerInputs inputs);

  public void setVoltage(double volt);
}
