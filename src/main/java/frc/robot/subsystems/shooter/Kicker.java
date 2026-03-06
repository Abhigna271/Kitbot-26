package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private KickerIO m_io;
  public final KickerInputsAutoLogged m_inputs = new KickerInputsAutoLogged();

  private frc.robot.util.SubsystemProfiles<KickerState> m_profiles;

  public static enum KickerState {
    kIdle,
    kSpinning,
  }

  public Kicker(KickerIO kickerIO) {
    m_io = kickerIO;
    Map<KickerState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(KickerState.kIdle, this::idlePeriodic);
    periodicHash.put(KickerState.kSpinning, this::spinningPeriodic);
    m_profiles = new frc.robot.util.SubsystemProfiles<>(periodicHash, KickerState.kIdle);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Kicker", m_inputs);
    Logger.recordOutput("Kicker/state", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(KickerConstants.kIdleVoltage.get());
  }

  public void spinningPeriodic() {
    m_io.setVoltage(KickerConstants.kSpinningVoltage.get());
  }

  public void updateState(KickerState state) {
    m_profiles.setCurrentProfile(state);
  }

  public KickerState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }
}
