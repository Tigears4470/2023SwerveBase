package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static final SwerveSubsystem swerveSub = new SwerveSubsystem();

  // INIT XBOX CONTROLLER
  public static XboxController xbox1 = new XboxController(0);

  // SMARTDASHBOARD
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // SHUFFLEBOARD
  private ShuffleboardTab shuffleDriverTab = Shuffleboard.getTab("Driver's Tab");

  // Create event map for Path Planner (there should only be one)
  public static final HashMap<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {
    // Configure default commands
    setDriveMode();

    // Configure controller bindings
    configureButtonBindings();

    // Configure auto
    String[] autoList = { "Do Nothing" };
    SmartDashboard.putStringArray("Auto List", autoList);
    initializeAutoChooser();

    // Initialize path planner event maps
    initializeEventMap();
  }

  // update shuffleboard layout
  public void updateShuffleboard() {
  }

  public void initializeAutoChooser() {
    // with command chooser
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    autoChooser = AutoBuilder.buildAutoChooser();
    shuffleDriverTab.add("Auto Routine", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void initializeEventMap() {
    eventMap.put("marker1", new PrintCommand("Pressed Marker 1"));
  }

  // assign button functions
  private void configureButtonBindings() {
  }

  public Command getAutoInput() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
  }

  public void setMotorBrake(boolean brake) {
  }
}