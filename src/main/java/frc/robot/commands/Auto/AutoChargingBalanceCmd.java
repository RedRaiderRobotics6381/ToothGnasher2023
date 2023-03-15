package frc.robot.commands.Auto;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
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

        chassisSpeeds = new ChassisSpeeds(1, 0, 0);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        // armSubsystem.intakeMotor.set(-0.05);
    }

    @Override
    public boolean isFinished() {
        System.out.println(swerveSubsystem.getPitch() + " Pitch Auto");
        if(swerveSubsystem.getPitch() < -15){
            return true;
        } else{
            return false;
        }
    }
}