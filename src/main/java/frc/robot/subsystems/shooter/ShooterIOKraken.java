package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.CurrentLimitConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.CtreBaseRefreshManager;


import java.util.List;

public class ShooterIOKraken implements ShooterIO {
  private TalonFX m_motor;

  private StatusSignal<ConnectedMotorValue> m_connectedMotor;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Temperature> m_motorTemperature;

  private final TalonFXConfiguration m_config;

  private VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public ShooterIOKraken(int Port, String Bus) {
    m_motor = new TalonFX(Port, Bus);

    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kShooterDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kShooterDefaultStatorLimit);

    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ShooterConstants.kGearRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfig)
            .withMotorOutput(motorOutput);

    m_motor.getConfigurator().apply(m_config);

    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorVelocity = m_motor.getVelocity();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorTemperature = m_motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_connectedMotor,
        m_motorVelocity,
        m_motorCurrent,
        m_motorStatorCurrent,
        m_motorVoltage,
        m_motorTemperature);

    if (Constants.kUseBaseRefreshManager) {
      CtreBaseRefreshManager.addSignals(
          List.of(
              m_connectedMotor,
              m_motorVelocity,
              m_motorCurrent,
              m_motorStatorCurrent,
              m_motorVoltage,
              m_motorTemperature));
    }
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
      BaseStatusSignal.refreshAll(
              null,
              m_connectedMotor,
              m_motorVelocity,
              m_motorCurrent,
              m_motorStatorCurrent,
              m_motorVoltage,
              m_motorTemperature)
          .isOK();
    }
    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;

    inputs.velocityRPS = m_motorVelocity.getValue().in(RotationsPerSecond);
    inputs.current = m_motorCurrent.getValue().in(Amps);
    inputs.statorCurrent = m_motorStatorCurrent.getValue().in(Amps);
    inputs.voltage = m_motorVoltage.getValue().in(Volts);
    inputs.temperature = m_motorTemperature.getValue().in(Celsius);
  }

  @Override
  public void setVoltage(double volt) {
    m_motor.setControl(m_voltageOut.withOutput(volt));
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }
}
