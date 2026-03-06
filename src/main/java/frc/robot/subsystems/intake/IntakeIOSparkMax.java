package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeIOSparkMax implements IntakeIO {
  private SparkMax m_motor;

  public IntakeIOSparkMax(int Port) {
    m_motor = new SparkMax(Port, MotorType.kBrushed);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.voltage = m_motor.getBusVoltage();
    inputs.current = m_motor.getOutputCurrent();
    inputs.output = m_motor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double volt) {
    m_motor.setVoltage(volt);
  }
}
