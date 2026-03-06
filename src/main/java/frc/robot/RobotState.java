package frc.robot;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
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
  private Shooter m_Shooter;



  public RobotState(Intake m_Intake, Shooter m_Shooter) {

    HashMap<RobotAction, Runnable> hash = new HashMap<>();
    hash.put(RobotAction.kAutoDefault, () -> {});
    hash.put(RobotAction.kTeleopDefault, () -> {});
    hash.put(RobotAction.kIntaking, () -> {});
    hash.put(RobotAction.kShooting, () -> {});

    m_profiles = new SubsystemProfiles<>(hash, RobotAction.kTeleopDefault);
    this.m_Intake = m_Intake;
    this.m_Shooter = m_Shooter;
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
        break;
      case kIntaking:
        break;
      case kShooting:
        m_Intake.updateState(IntakeState.kRev);
        m_Shooter.updateState(ShooterState.kSpinning);
      default:
        break;
    }

    m_profiles.setCurrentProfile(action);
  }

  public static RobotState getInstance() {
    return m_instance;
  }

  public static RobotState startInstance() {
    if (m_instance == null) {
      m_instance = new RobotState();
    }
    return m_instance;
  }

  public RobotAction getCurrAction() {
    return m_profiles.getCurrentProfile();
  }
}
