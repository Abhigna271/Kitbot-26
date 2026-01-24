package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;

  public IntakeIOSim() {

    var plant =
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.kSimGearbox, IntakeConstants.kSimMOI, IntakeConstants.kSimGearing);
    m_sim = new DCMotorSim(plant, IntakeConstants.kSimGearbox);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.velocityRPS = m_sim.getAngularVelocityRPM() / 60;
    inputs.accelerationRPSSq = Units.radiansToRotations(m_sim.getAngularAccelerationRadPerSecSq());
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.voltage = m_voltage;
    inputs.motorIsConnected = true;
  }

  @Override
  public void setVoltage(double volt) {
    m_voltage = volt;
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {}
}
