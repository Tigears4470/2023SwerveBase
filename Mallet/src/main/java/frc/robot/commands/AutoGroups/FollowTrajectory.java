package frc.robot.commands.AutoGroups;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FollowTrajectory extends SequentialCommandGroup {
    public FollowTrajectory(SwerveDriveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(drivebase);
        if (resetOdometry)
            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        addCommands(
                // Command to follow trajectory
                new PPSwerveControllerCommand(
                        trajectory,
                        drivebase::getPose,
                        Auton.xAutoPID.createPIDController(),
                        Auton.yAutoPID.createPIDController(),
                        Auton.angleAutoPID.createPIDController(),
                        drivebase::setChassisSpeeds,
                        drivebase));
    }
}
