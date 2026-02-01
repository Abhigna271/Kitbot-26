package frc.robot.subsystems.intake;

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
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.CtreBaseRefreshManager;
import java.util.List;

// Contains general hardware, motor, statussignals, etc
public class IntakeIOKraken implements IntakeIO {
  private TalonFX m_motor;

  private StatusSignal<ConnectedMotorValue> m_connectedMotor;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<AngularVelocity> m_motorVelocity;
  private StatusSignal<Current> m_motorCurrent;
  private StatusSignal<Current> m_motorStatorCurrent;
  private StatusSignal<Temperature> m_motorTemperature;

  // instance variable
  private final TalonFXConfiguration m_config;

  /*creates a new voltage output to track
  FOC controls torque*/
  private VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  // constructor
  public IntakeIOKraken(int Port, String Bus) {
    // initializing motor
    m_motor = new TalonFX(Port, Bus);

    // object that stores all of the limits for the motor
    var currentLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeDefaultSupplyLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(CurrentLimitConstants.kIntakeDefaultStatorLimit);
    // Assign gear ratio
    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kGearRatio);
    // setting the motor usage to 0 when power is not being added (brakes if control is alone)
    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    // setting the new motor as m_config and setting limits, rotation feedback, and motor output
    m_config =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfig)
            .withMotorOutput(motorOutput);

    // Configurater is used to change settings
    m_motor.getConfigurator().apply(m_config);

    // Gets all of the different variables
    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorVelocity = m_motor.getVelocity();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorTemperature = m_motor.getDeviceTemp();

    // Sets amount of times for update, but doesn't actually do it
    BaseStatusSignal.setUpdateFrequencyForAll(
        75.0,
        m_connectedMotor,
        m_motorVelocity,
        m_motorCurrent,
        m_motorStatorCurrent,
        m_motorVoltage,
        m_motorTemperature);

    // What gets updated, not actually updating anything
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

  // Actually refreshes inputs
  @Override
  public void updateInputs(IntakeInputs inputs) {
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

    // Setting the values and units
    inputs.motorIsConnected = m_connectedMotor.getValue() != ConnectedMotorValue.Unknown;
    inputs.velocityRPS = m_motorVelocity.getValue().in(RotationsPerSecond);
    inputs.current = m_motorCurrent.getValue().in(Amps);
    inputs.statorCurrent = m_motorStatorCurrent.getValue().in(Amps);
    inputs.voltage = m_motorVoltage.getValue().in(Volts);
    inputs.temperature = m_motorTemperature.getValue().in(Celsius);
  }
  // adds an amount of voltage to motor at a moment
  @Override
  public void setVoltage(double volt) {
    m_motor.setControl(m_voltageOut.withOutput(volt));
  }

  // add the current limits etc to the motor "m_config"
  @Override
  public void setCurrentLimits(double supplyLimit) {
    m_motor
        .getConfigurator()
        .apply(m_config.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
  }
}
