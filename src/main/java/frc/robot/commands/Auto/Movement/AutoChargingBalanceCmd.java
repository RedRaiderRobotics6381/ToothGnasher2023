package frc.robot.commands.Auto.Movement;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.Primary.SwerveSubsystem;
import frc.robot.subsystems.Secondary.ArmSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoChargingBalanceCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    double ySpeed;

    public AutoChargingBalanceCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds;

        
        // Maybe -
        chassisSpeeds = new ChassisSpeeds(SensorConstants.PIDspeed.calculate(0, swerveSubsystem.getYaw()), 0, 0);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // if(Math.abs(swerveSubsystem.getYaw()) < 0.25)
        return false;
    }
}