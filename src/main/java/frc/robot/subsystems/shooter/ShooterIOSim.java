package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;

  public ShooterIOSim() {

    var plant =
        LinearSystemId.createDCMotorSystem(
            ShooterConstants.kSimGearbox, ShooterConstants.kSimMOI, ShooterConstants.kSimGearing);
    m_sim = new DCMotorSim(plant, ShooterConstants.kSimGearbox);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
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

  @Override
  public void setPIDFF(double kP, double kI, double kD, double kS) {}
}
