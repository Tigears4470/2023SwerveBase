package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.GyroConstants;
import swervelib.imu.NavXSwerve;

public class GyroIONavX implements GyroIO {
    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);
}