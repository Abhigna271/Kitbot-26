package frc.robot.subsystems.Kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private KickerIO m_io;
  public final KickerInputsAutoLogged m_inputs = new KickerInputsAutoLogged();
  private SubsystemProfiles<KickerState> m_profiles;
  // creates the states in intake

  public static enum KickerState {
    kIdle,
    kIntaking,
    kShooting
    // list of states there
  }

  public Kicker(KickerIO kickerIO) {
    // creating an intake that takes in one of the IO files
    m_io = kickerIO;
    Map<KickerState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(KickerState.kIdle, this::idlePeriodic);
    periodicHash.put(KickerState.kShooting, this::kShootingPeriodic);
    periodicHash.put(KickerState.kIntaking, this::intakingPeriodic);
    // creating map/connecting the variables called periodic hash
    // Runnable creates "threading"/runs it at the same time

    m_profiles = new SubsystemProfiles<>(periodicHash, KickerState.kIdle);
    // making profiles the hash map, then default state
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

  public void intakingPeriodic() {
    m_io.setVoltage(KickerConstants.kIntakingVoltage.get());
  }

  public void kShootingPeriodic() {
    m_io.setVoltage(KickerConstants.kShootingVoltage.get());
  }

  public void updateState(KickerState state) {
    Logger.recordOutput("KickerStatehasupdated", true);
    m_profiles.setCurrentProfile(state);
  }

  public KickerState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }
}
