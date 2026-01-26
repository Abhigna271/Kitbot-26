package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO m_io;
  public final ShooterInputsAutoLogged m_inputs = new ShooterInputsAutoLogged();

    private frc.robot.util.SubsystemProfiles<ShooterState> m_profiles;

  public static enum ShooterState {
    kIdle,
    kSpinning,
  }

  public Shooter(ShooterIO shooterIO) {
    m_io = shooterIO;
    Map<ShooterState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(ShooterState.kIdle, this::idlePeriodic);
    periodicHash.put(ShooterState.kSpinning, this::spinningPeriodic);

        m_profiles = new frc.robot.util.SubsystemProfiles<>(periodicHash, ShooterState.kIdle);
    }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Shooter", m_inputs);
    Logger.recordOutput("Shooter/state", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(ShooterConstants.kIdleVoltage.get());
  }

  public void spinningPeriodic() {
    m_io.setVoltage(ShooterConstants.kSpinningVoltage.get());
  }

  public ShooterState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }

  public double getCurrentVelocity() {
    return m_inputs.velocityRPS;
  }
}