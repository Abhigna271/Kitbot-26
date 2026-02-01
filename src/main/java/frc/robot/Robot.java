// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.AlertManager;
import frc.robot.util.CtreBaseRefreshManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  private WPILOGWriter m_writer = new WPILOGWriter();
  private Alert m_writerAlert = new Alert("WPILOGWriter Failed to start", AlertType.kError);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.kCurrentMode) {
      case REAL:
      case PROTO:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(m_writer);
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        // setUseTiming(true);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    if (!Constants.kUsePhoenixDiagnosticServer) {
      Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    }

    // Set up robot container
    m_robotContainer = new RobotContainer();

    AlertManager.registerAlert(m_writerAlert);

    DriverStation.silenceJoystickConnectionWarning(true);

    SignalLogger.enableAutoLogging(false);
  }

  @Override
  public void robotPeriodic() {
    if (Constants.kUseBaseRefreshManager) {
      CtreBaseRefreshManager.updateAll();
    }

    if (Constants.kUseAlerts) {
      // I LOVE REFLECTION
      boolean isOpen = true;
      try {
        var openField = m_writer.getClass().getDeclaredField("isOpen");
        openField.setAccessible(true);
        isOpen = (Boolean) openField.get(m_writer);
      } catch (Exception e) {
        e.printStackTrace();
      }
      if (isOpen == false) {
        m_writerAlert.set(true);
      } else {
        m_writerAlert.set(false);
      }
    }
    CommandScheduler.getInstance().run();

    AlertManager.update();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
