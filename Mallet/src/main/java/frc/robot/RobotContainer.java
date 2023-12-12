package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.AbsoluteDrive;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.PrintCommand;


import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;

public class RobotContainer {
  // INIT SUBSYSTEMS
  private static final Drivetrain m_drivetrain = new Drivetrain(new File(Filesystem.getDeployDirectory(), "neo/swerve"));
  
    // Event Map for Path Planner, there should only be one to handle all the event 
    public static final HashMap<String, Command> m_eventMap = new HashMap<>();
  
  // INIT XBOX CONTROLLER
  public static CommandXboxController m_xbox = new CommandXboxController(0);

  // INIT CONTROLER ARRAYS
  public static HashMap<String, Trigger> controllerButtons = new HashMap<String, Trigger>();

  // SMARTDASHBOARD
  private SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // SHUFFLEBOARD
  private ShuffleboardTab main = Shuffleboard.getTab("Driver's Tab");

  public RobotContainer() {
    configureButtonBindings();

    m_drivetrain.initAutoBuilder(m_eventMap, new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), false);
    
    String[] autoList = {"Do Nothing"};
    SmartDashboard.putStringArray("Auto List", autoList);
    
    initializeEventMap();
    initializeAutoChooser();
    
    System.out.print(I2C.Port.kMXP);
  }

  // update shuffleboard layout
  public void updateShuffleboard() {
  }

  public void initializeAutoChooser() {
    // with string chooser
    
    // with command chooser
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));
    m_autoChooser.addOption("Two Meter Path", m_drivetrain.loadPathPlannerCommand("TwoMeterPath", new PathConstraints(Constants.K_MAX_VELOCITY, Constants.K_MAX_ACCEL)));
    m_autoChooser.addOption("Event One Path", m_drivetrain.loadPathPlannerCommand("EventOnePath", new PathConstraints(Constants.K_MAX_VELOCITY, Constants.K_MAX_ACCEL)));
    main.add("Auto Routine", m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

  }

  public void initializeEventMap(){
    m_eventMap.put("marker1", new PrintCommand("Pressed Marker 1"));
  }

  // assign button functions
  private void configureButtonBindings() {
    m_drivetrain.setDefaultCommand(new AbsoluteDrive(m_drivetrain, () -> MathUtil.applyDeadband(-m_xbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), () -> MathUtil.applyDeadband(-m_xbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), () -> MathUtil.applyDeadband(-m_xbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), () -> MathUtil.applyDeadband(-m_xbox.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND), false));
    
    m_xbox.a().onTrue(m_drivetrain.loadPathPlannerCommand("TwoMeterPath", new PathConstraints(Constants.K_MAX_VELOCITY, Constants.K_MAX_ACCEL)));
    m_xbox.b().onTrue(m_drivetrain.loadPathPlannerCommand("EventOnePath", new PathConstraints(Constants.K_MAX_VELOCITY, Constants.K_MAX_ACCEL)));
  }

  public Command getAutoInput() {
    return m_autoChooser.getSelected();
  }
}