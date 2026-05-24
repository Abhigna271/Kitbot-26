package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO m_io;
  public final IntakeInputsAutoLogged m_inputs = new IntakeInputsAutoLogged();

  private frc.robot.util.SubsystemProfiles<IntakeState> m_profiles;

  public static enum IntakeState {
    kIdle,
    kSpinning,
  }

  public Intake(IntakeIO intakeIO) {
    m_io = intakeIO;
    Map<IntakeState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IntakeState.kIdle, this::idlePeriodic);
    periodicHash.put(IntakeState.kSpinning, this::spinningPeriodic);
    m_profiles = new frc.robot.util.SubsystemProfiles<>(periodicHash, IntakeState.kIdle);
    if (Robot.isReal()) {
      m_io.setPIDFF(
          IntakeConstants.kShooterP.get(),
          IntakeConstants.kShooterI.get(),
          IntakeConstants.kShooterD.get(),
          IntakeConstants.kShooterKS.get());
    }
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    m_profiles.getPeriodicFunctionTimed().run();

    Logger.processInputs("Intake", m_inputs);
    Logger.recordOutput("Intake/state", m_profiles.getCurrentProfile());
  }

  public void idlePeriodic() {
    m_io.setVoltage(IntakeConstants.kIdleVoltage.get());
  }

  public void spinningPeriodic() {
    m_io.setVoltage(IntakeConstants.kSpinningVoltage.get());
  }

  public void updateState(IntakeState state) {
    m_profiles.setCurrentProfile(state);
  }

  public IntakeState getCurrentState() {
    return m_profiles.getCurrentProfile();
  }
}
