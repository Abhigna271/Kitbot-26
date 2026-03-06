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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Kicker;
import frc.robot.subsystems.shooter.KickerIOSim;
import frc.robot.subsystems.shooter.KickerIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private Intake m_intake;

  private Kicker m_shooter;
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
      m_shooter = new Kicker(new KickerIOSparkMax(Ports.kShooter));
      m_drive = new Drive(new DriveIOCIM());
    } else {
      m_intake = new Intake(new IntakeIOSim());
      m_shooter = new Kicker(new KickerIOSim());
      m_drive = new Drive(new DriveIOCIM());
    }
  }

  public void configureCommands() {
    RobotState.startInstance(m_intake, m_shooter);
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
                  if (m_intake.getCurrentState() != Intake.IntakeState.kIntaking) {
                    m_intake.updateState(IntakeState.kIntaking);
                  } else {
                    m_intake.updateState(IntakeState.kIdle);
                  }
                }));
    m_controller
        .rev()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (m_intake.getCurrentState() != Intake.IntakeState.kShooting) {
                    m_intake.updateState(IntakeState.kShooting);
                  } else {
                    m_intake.updateState(IntakeState.kIdle);
                  }
                }));
    m_controller
        .shoot()
        .onTrue(
            Commands.runOnce(
                () -> {
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
