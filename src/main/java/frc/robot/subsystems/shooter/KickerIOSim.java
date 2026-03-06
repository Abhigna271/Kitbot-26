package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.KickerConstants;

public class KickerIOSim implements KickerIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;

  public KickerIOSim() {

    var plant =
        LinearSystemId.createDCMotorSystem(
            KickerConstants.kSimGearbox, KickerConstants.kSimMOI, KickerConstants.kSimGearing);
    m_sim = new DCMotorSim(plant, KickerConstants.kSimGearbox);
  }

  @Override
  public void updateInputs(KickerInputs inputs) {
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    inputs.voltage = m_sim.getInputVoltage();
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.output = m_sim.getOutput(0);
  }

  @Override
  public void setVoltage(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public void setCurrentLimits(double supplyLimit) {}
}
