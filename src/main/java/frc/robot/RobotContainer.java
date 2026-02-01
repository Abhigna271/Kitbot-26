package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private Intake m_intake;

  private Shooter m_shooter;
  // Controller
  private DriverControls m_controller;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  public RobotContainer() {
    configureSubsystems();
    configureCommands();
    configureControllers();
    configureBindings();
  }

  public void configureSubsystems() {

    if (RobotBase.isReal()) {
      m_intake = new Intake(new IntakeIOKraken(Ports.kIntake, Ports.kIntakeCanivoreName));
      m_shooter = new Shooter(new ShooterIOKraken(Ports.kShooter, Ports.kShooterCanivoreName));
    } else {
      m_intake = new Intake(new IntakeIOSim());
      m_shooter = new Shooter(new ShooterIOSim());
    }
  }

  public void configureCommands() {}

  public void configureControllers() {
    m_controller = new DriverControlsPS5(0);
  }

  public void configureBindings() {
    m_controller
        .intake()
        .onTrue(
            Commands.runOnce(
                (() -> {
                  if (m_intake.getCurrentState() != Intake.IntakeState.kIdle) {
                    m_intake.updateState(Intake.IntakeState.kIdle);
                  } else {
                    m_intake.updateState(Intake.IntakeState.kSpinning);
                  }
                })));


    m_controller
        .shooter()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_shooter.updateState(ShooterState.kSpinning);
                }));

    m_controller
        .shooter()
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_shooter.updateState(ShooterState.kIdle);
                }));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public String getSelectedAuto() {
    return m_autoChooser.getSendableChooser().getSelected();
  }
}
