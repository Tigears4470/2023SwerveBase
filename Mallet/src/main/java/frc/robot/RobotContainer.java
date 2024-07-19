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

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  // INIT SUBSYSTEMS
  // private static final SwerveSubsystem swerveSub = new SwerveSubsystem();

  // INIT XBOX CONTROLLER

  // SMARTDASHBOARD

  // SHUFFLEBOARD
  private ShuffleboardTab shuffleDriverTab = Shuffleboard.getTab("Driver's Tab");

  // Create event map for Path Planner (there should only be one)
  public static final HashMap<String, Command> eventMap = new HashMap<>();

  public RobotContainer() {
    if(Robot.isReal()) {
      Logger.addDataReceiver(null);
    }



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
    
  }

  public void initializeEventMap() {
    eventMap.put("marker1", new PrintCommand("Pressed Marker 1"));
  }

  // assign button functions
  private void configureButtonBindings() {
  }

  public Command getAutoInput() {
    // return autoChooser.getSelected();
    return null;
  }

  public void setDriveMode() {
  }

  public void setMotorBrake(boolean brake) {
  }
}