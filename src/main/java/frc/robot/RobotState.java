package frc.robot;

import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public enum RobotAction {
    kAutoDefault,
    kTeleopDefault,
    kIntaking
  }

  private SubsystemProfiles<RobotAction> m_profiles;
  private static RobotState m_instance;

  public RobotState() {
  
    HashMap<RobotAction, Runnable> hash = new HashMap<>();
    hash.put(RobotAction.kAutoDefault, () -> {});
    hash.put(RobotAction.kTeleopDefault, () -> {});
    hash.put(RobotAction.kIntaking, () -> {});

    m_profiles = new SubsystemProfiles<>(hash, RobotAction.kTeleopDefault);
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
