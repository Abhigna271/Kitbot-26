package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class KickerIOSparkMax implements KickerIO {
  private SparkMax m_motor;

  public KickerIOSparkMax(int Port) {
    m_motor = new SparkMax(Port, MotorType.kBrushed);
  }

  @Override
  public void updateInputs(KickerInputs inputs) {
    inputs.voltage = m_motor.getBusVoltage();
    inputs.current = m_motor.getOutputCurrent();
    inputs.output = m_motor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double volt) {
    m_motor.setVoltage(volt);
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {}
}
