package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Ports;
import frc.robot.RobotState.RobotAction;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private Drive m_drive;
  private Intake m_intake;

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
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(0),
              new ModuleIOTalonFX(1),
              new ModuleIOTalonFX(2),
              new ModuleIOTalonFX(3));
      m_intake = new Intake(new IntakeIOKraken(Ports.kIntake));
    } else {
      m_drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
      m_intake = new Intake(new IntakeIOSim());
    }
  }

  public void configureCommands() {
    RobotState.startInstance(m_drive, m_intake);
  }

  public void configureControllers() {
    m_controller = new DriverControlsPS5(0);
  }

  public void configureBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            m_controller::getForward,
            m_controller::getStrafe,
            m_controller::getTurn,
            false));
    m_controller
        .resetFieldCentric()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_drive.setPose(new Pose2d());
                }));
    m_controller
        .intake()
        .onTrue(
            Commands.runOnce(
                (() -> {
                  if(RobotState.getInstance().getCurrAction() != RobotAction.kIntaking) {
                  RobotState.getInstance().updateRobotAction(RobotAction.kIntaking);
                  } else {
                    RobotState.getInstance().updateRobotAction(RobotAction.kTeleopDefault);
                  }
                })));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public String getSelectedAuto() {
    return m_autoChooser.getSendableChooser().getSelected();
  }
}
