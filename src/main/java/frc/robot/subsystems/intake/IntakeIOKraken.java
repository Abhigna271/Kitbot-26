package frc.robot.subsystems.intake;

import java.util.List;
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

public class IntakeIOKraken implements IntakeIO {
    private TalonFX m_motor;
    
    private StatusSignal<ConnectedMotorValue> m_connectedMotor;
    private StatusSignal<Voltage> m_motorVoltage;
    private StatusSignal<AngularVelocity> m_motorVelocity;
    private StatusSignal<Current> m_motorCurrent;
    private StatusSignal<Current> m_motorStatorCurrent;
    private StatusSignal<Temperature> m_motorTemperature;

    private final TalonFXConfiguration m_config; 

    private VoltageOut m_voltageOut = new VoltageOut(0.0).withEnableFOC(true);

    public IntakeIOKraken(int Port, String Bus){
        m_motor = new TalonFX(Port, Bus);

    var currentLimits =
    new CurrentLimitsConfigs()
    .withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(CurrentLimitConstants.kIntakeDefaultSupplyLimit)
    .withStatorCurrentLimitEnable(true)
    .withStatorCurrentLimit(CurrentLimitConstants.kIntakeDefaultStatorLimit);
//ASK ABOUT THIS 
    var feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.kGearRatio);

    var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    m_config =
        new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withFeedback(feedbackConfig)
        .withMotorOutput(motorOutput);
//Configurater is used to change settings
    m_motor.getConfigurator().apply(m_config);
//Gets all of the different variables
    m_connectedMotor = m_motor.getConnectedMotor();
    m_motorVelocity = m_motor.getVelocity();
    m_motorCurrent = m_motor.getSupplyCurrent();
    m_motorStatorCurrent = m_motor.getStatorCurrent();
    m_motorVoltage = m_motor.getMotorVoltage();
    m_motorTemperature = m_motor.getDeviceTemp();
//Sets amount of times for update, but doesn't actually do it
    BaseStatusSignal.setUpdateFrequencyForAll(
    75.0,
     m_connectedMotor,
    m_motorVelocity,
    m_motorCurrent,
    m_motorStatorCurrent,
    m_motorVoltage,
    m_motorTemperature);
//What gets updated, not actually updating anything
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
public void updateInputs(IntakeInputs inputs) {
    if (!Constants.kUseBaseRefreshManager) {
        BaseStatusSignal.refreshAll(
            null,
             m_connectedMotor,
             m_motorVelocity,
             m_motorCurrent,
             m_motorStatorCurrent,
             m_motorVelocity,
             m_motorTemperature)
             .isOK();

    }
}
    
@Override
    public void setVoltage(double volt) {
}

@Override
 public void setCurrentLimits(double supplyLimit) {  
 }
}


