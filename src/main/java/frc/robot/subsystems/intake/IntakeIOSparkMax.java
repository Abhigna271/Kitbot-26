package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSparkMax implements IntakeIO {
  private SparkMax m_motor;
  private SparkClosedLoopController m_Controller = m_motor.getClosedLoopController();
  private SparkMaxConfig m_Config = new SparkMaxConfig();

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
    m_Controller.setSetpoint(volt, ControlType.kCurrent);
  }

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS) {
    m_Config.closedLoop.p(kP).i(kI).d(kD);
    m_Config.closedLoop.feedForward.kS(kS);
  }
}
