package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private DCMotorSim m_sim;
  private double m_voltage = 0.0;
  // Theoretical version of Robot
  public IntakeIOSim() {

    // creating fake motor and adding constants to it
    var plant =
        LinearSystemId.createDCMotorSystem(
            IntakeConstants.kSimGearbox, IntakeConstants.kSimMOI, IntakeConstants.kSimGearing);
    m_sim = new DCMotorSim(plant, IntakeConstants.kSimGearbox);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    // sets voltage and makes the simulating "update" every 0.02 sec
    m_sim.setInputVoltage(m_voltage);
    m_sim.update(0.02);

    // sets simulation statistics and auto data
    inputs.voltage = m_sim.getInputVoltage();
    inputs.current = m_sim.getCurrentDrawAmps();
    inputs.output = m_sim.getOutput(0);

  }

  @Override
  public void setVoltage(double volt) {
    m_voltage = volt;
  }
}
