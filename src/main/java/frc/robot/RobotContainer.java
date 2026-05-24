package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.DriveIOCIM;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOSparkMax;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Kicker.KickerIOSim;
import frc.robot.subsystems.Kicker.KickerIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private Intake m_intake;

  private Kicker m_kicker;
  // Controller
  private DriverControls m_controller;

  private Drive m_drive;

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
      m_intake = new Intake(new IntakeIOSparkMax(Ports.kIntake));
      m_kicker = new Kicker(new KickerIOSparkMax(Ports.kShooter));
      m_drive = new Drive(new DriveIOCIM());
    } else {
      m_intake = new Intake(new IntakeIOSim());
      m_kicker = new Kicker(new KickerIOSim());
      m_drive = new Drive(new DriveIOCIM());
    }
  }

  public void configureCommands() {
    RobotState.startInstance(m_intake, m_kicker);
  }

  public void configureControllers() {
    m_controller = new DriverControlsPS5(0);
  }

  public void configureBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.arcadeDrive(m_drive, m_controller::getMovement, m_controller::getRotation));
    m_controller
        .intake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  org.littletonrobotics.junction.Logger.recordOutput("Intake/fires", true);
                  if (m_intake.getCurrentState() != Intake.IntakeState.kSpinning) {
                    m_intake.updateState(IntakeState.kSpinning);
                  } else {
                    m_intake.updateState(IntakeState.kIdle);
                  }
                }));
    m_controller
        .rev()
        .onTrue(
            Commands.runOnce(
                () -> {
                  org.littletonrobotics.junction.Logger.recordOutput("shoot/fires", true);
                  if (m_intake.getCurrentState() != Intake.IntakeState.kSpinning) {
                    m_intake.updateState(IntakeState.kSpinning);
                  } else {
                    m_intake.updateState(IntakeState.kIdle);
                  }
                }));
    m_controller
        .shoot()
        .onTrue(
            Commands.runOnce(
                () -> {
                  org.littletonrobotics.junction.Logger.recordOutput("shooter/fires", true);
                  if (RobotState.getInstance().getCurrAction() != RobotAction.kShooting) {
                    RobotState.getInstance().updateRobotAction(RobotAction.kShooting);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kTeleopDefault);
                  }
                }));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public String getSelectedAuto() {
    return m_autoChooser.getSendableChooser().getSelected();
  }
}
