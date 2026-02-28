package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {
    private SparkMax m_motor;
    
    private SparkClosedLoopController m_Controller = m_motor.getClosedLoopController();
    private SparkMaxConfig m_Config = new SparkMaxConfig();

    public ShooterIOSparkMax(int Port){
        m_motor = new SparkMax(Port, MotorType.kBrushed);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.voltage = m_motor.getBusVoltage();
        inputs.current = m_motor.getOutputCurrent();
        inputs.output = m_motor.getAppliedOutput();
}

    @Override
    public void setVoltage(double volt) {
        m_Controller.setSetpoint(volt, ControlType.kCurrent);
        }

@Override 
public void setCurrentLimits(double supplyLimit) {
}

@Override
public void setPIDFF(double kP, double kI, double kD, double kS) {
    m_Config.closedLoop.p(kP).i(kI).d(kD);
    m_Config.closedLoop.feedForward.kS(kS);


}

}

