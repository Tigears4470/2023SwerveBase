package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  // Subsystem Init
  private static final SwerveSubsystem swerveSub = new SwerveSubsystem();

  // Controllers
  private static CommandXboxController xbox1 = new CommandXboxController(0);

  private LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("AutoChooser");
  // private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // runs only when the robot is first started
  @Override
  public void robotInit() {
    Logger.recordMetadata("SwerveBase", null);

    if(isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  }

  // called periodically regardless of mode
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // called when the robot is disabled
  @Override
  public void disabledInit() {
  }

  // called continuously while robot is disabled
  @Override
  public void disabledPeriodic() {
  }

  // called when auto is selected
  @Override
  public void autonomousInit() {
  }

  // called periodically when robot is in auto
  @Override
  public void autonomousPeriodic() {
  }

  // called when teleop is selected
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // called periodically when robot is in teleop
  @Override
  public void teleopPeriodic() {
  }

  // called when test is selected
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // called periodically when robot is in test
  @Override
  public void testPeriodic() {
  }
}
