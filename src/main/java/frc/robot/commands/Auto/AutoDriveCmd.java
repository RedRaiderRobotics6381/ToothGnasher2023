package frc.robot.commands.Auto;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoDriveCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private SwerveModuleState[] states;
    double value;

    public AutoDriveCmd(SwerveSubsystem swerveSubsystem, double value) {
        this.value = value;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = new ChassisSpeeds(0, 0.5, 0);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // states = moduleStates;
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.setModuleStates(states);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}