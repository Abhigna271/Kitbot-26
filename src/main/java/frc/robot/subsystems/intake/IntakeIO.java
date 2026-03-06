package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
// IO is js the blueprint, showing all the necessities of an Intake file

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double voltage;
    public double current;
    public double output;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double volt);

  public void setPIDFF(double kP, double kI, double kD, double kS);
}
