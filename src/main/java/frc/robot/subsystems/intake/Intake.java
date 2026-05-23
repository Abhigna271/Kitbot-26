package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.util.SubsystemProfiles;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO m_io;
  // new IntakeIO called m_io
  public final IntakeInputsAutoLogged m_inputs = new IntakeInputsAutoLogged();
  // m_inputs is a new logged category

  private SubsystemProfiles<IntakeState> m_profiles;
  // creates the states in intake

  public static enum IntakeState {
    kIdle,
    kShooting,
    kIntaking
    // list of states there
  }

  public Intake(IntakeIO intakeIO) {
    // creating an intake that takes in one of the IO files
    m_io = intakeIO;
    Map<IntakeState, Runnable> periodicHash = new HashMap<>();
    periodicHash.put(IntakeState.kIdle, this::idlePeriodic);
    periodicHash.put(IntakeState.kIntaking, this::intakingPeriodic);
    // creating map/connecting the variables called periodic hash
    // Runnable creates "threading"/runs it at the same time

    m_profiles = new SubsystemProfiles<>(periodicHash, IntakeState.kIdle);
    // making profiles the hash map, then default state

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
    // update inputs in the 3 classes
    m_profiles.getPeriodicFunctionTimed().run();
    // get whatever mapped into current state, runs it
    Logger.processInputs("Intake", m_inputs);
    Logger.recordOutput("Intake/state", m_profiles.getCurrentProfile());
    // updates and shows
  }

  public void idlePeriodic() {
    m_io.setVoltage(IntakeConstants.kIdleVoltage.get());
    // What to do during periodic
  }


  public void intakingPeriodic() {
    m_io.setVoltage(IntakeConstants.kIntakingVoltage.get());
  }

  public void updateState(IntakeState state) {
    Logger.recordOutput("IntakeStatehasupdated", true);
    m_profiles.setCurrentProfile(state);
    // allows to switch between states
  }

  public IntakeState getCurrentState() {
    return m_profiles.getCurrentProfile();
    // tells what its in
  }
}
