package frc.robot;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.Kicker.KickerState;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public enum RobotAction {
    kAutoDefault,
    kTeleopDefault,
    kIntaking,
    kShooting,
  }

  private SubsystemProfiles<RobotAction> m_profiles;
  private static RobotState m_instance;
  private Intake m_Intake;
  private Kicker m_Kicker;

  public RobotState(Intake m_Intake, Kicker m_Kicker) {

    HashMap<RobotAction, Runnable> hash = new HashMap<>();
    hash.put(RobotAction.kAutoDefault, () -> {});
    hash.put(RobotAction.kTeleopDefault, () -> {});
    hash.put(RobotAction.kIntaking, () -> {});
    hash.put(RobotAction.kShooting, () -> {});

    m_profiles = new SubsystemProfiles<>(hash, RobotAction.kTeleopDefault);
    this.m_Intake = m_Intake;
    this.m_Kicker = m_Kicker;
  }

  public void updateRobotState() {
    m_profiles.getPeriodicFunctionTimed().run();
    Logger.recordOutput("RobotAction", m_profiles.getCurrentProfile());
  }

  public void updateRobotAction(RobotAction action) {

    switch (action) {
      case kAutoDefault:
        break;
      case kTeleopDefault:
        m_Intake.updateState(IntakeState.kIdle);
        m_Kicker.updateState(KickerState.kIdle);
      case kIntaking:
        m_Intake.updateState(IntakeState.kIntaking);
        m_Kicker.updateState(KickerState.kIdle);
      case kShooting:
        m_Intake.updateState(IntakeState.kShooting);
        m_Kicker.updateState(KickerState.kSpinning);
      default:
        break;
    }

    m_profiles.setCurrentProfile(action);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance(Intake m_Intake, Kicker m_Kicker) {
    if (m_instance == null) {
      m_instance = new RobotState(m_Intake, m_Kicker);
    }
    return m_instance;
  }

  public RobotAction getCurrAction() {
    return m_profiles.getCurrentProfile();
  }
}
