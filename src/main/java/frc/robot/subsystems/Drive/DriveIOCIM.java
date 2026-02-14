package frc.robot.subsystems.Drive;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class DriveIOCIM implements DriveIO {
  private SparkMax m_LF;
  private SparkMax m_LR;
  private SparkMax m_RF;
  private SparkMax m_RR;

  public DriveIOCIM() {
    m_LF = new SparkMax(Constants.Ports.kLFPort, MotorType.kBrushed);
    m_LR = new SparkMax(Constants.Ports.kLRPort, MotorType.kBrushed);
    m_RF = new SparkMax(Constants.Ports.kRFPort, MotorType.kBrushed);
    m_RR = new SparkMax(Constants.Ports.kRRPort, MotorType.kBrushed);

    this.configureMotors();
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.voltage = new double[] {m_LF.getBusVoltage() * m_LF.getAppliedOutput()};
    inputs.current = new double[] {m_LF.getOutputCurrent()};
  }

  @Override
  public void setLeft(double value) {
    m_LF.set(value);
  }

  @Override
  public void setRight(double value) {
    m_LR.set(value);
  }

  private void configureMotors() {
    SparkMaxConfig m_LFConfig = new SparkMaxConfig();
    m_LFConfig.idleMode(IdleMode.kCoast);
    m_LFConfig.inverted(false);
    m_LF.configure(
        m_LFConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // front left

    SparkMaxConfig m_LRConfig = new SparkMaxConfig();
    m_LRConfig.idleMode(IdleMode.kCoast);
    m_LRConfig.inverted(true);
    m_LR.configure(
        m_LRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // back left

    SparkMaxConfig m_RFConfig = new SparkMaxConfig();
    m_RFConfig.idleMode(IdleMode.kCoast);
    m_RFConfig.inverted(false);
    m_RF.configure(
        m_RFConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // front right

    SparkMaxConfig m_RRConfig = new SparkMaxConfig();
    m_RRConfig.idleMode(IdleMode.kCoast);
    m_RRConfig.inverted(false);
    m_RR.configure(
        m_RRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // back right
  }
}
