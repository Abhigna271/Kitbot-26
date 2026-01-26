package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO m_Io;
  public final IntakeInputsAutoLogged m_inputs = new IntakeInputsAutoLogged();

    private frc.robot.util.SubsystemProfiles<IntakeState> m_Profiles;

  public static enum IntakeState {
    kIdle,
    kSpinning,
  }

  public Intake(IntakeIO intakeIO) {
    m_Io = intakeIO;
    Map<IntakeState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IntakeState.kIdle, this::idlePeriodic);
    periodicHash.put(IntakeState.kSpinning, this::spinningPeriodic);

        m_Profiles = new frc.util.SubsystemProfiles<Intake.IntakeState>(periodicHash, IntakeState.kIdle);
    }

  @Override
  public void periodic() {
    m_Io.updateInputs(m_inputs);
    m_Profiles.getPeriodicFunctionTimed().run();
    Logger.processInputs("Intake", m_inputs);
    Logger.recordOutput("Intake/state", m_Profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_Io.setVoltage(IntakeConstants.kIdleVoltage.get());
  }

  public void spinningPeriodic() {
    m_Io.setVoltage(IntakeConstants.kSpinningVoltage.get());
  }

  public void updateState(IntakeState state) {
    m_Profiles.setCurrentProfile(state);
  }

  public IntakeState getCurrenState() {
    return m_Profiles.getCurrentProfile();
  }

  public double getCurrentVelocity() {
    return m_inputs.velocityRPS;
  }
}
