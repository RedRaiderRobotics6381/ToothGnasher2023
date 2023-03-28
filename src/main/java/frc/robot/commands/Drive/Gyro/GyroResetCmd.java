package frc.robot.commands.Drive.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Primary.SwerveSubsystem;

public class GyroResetCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;

    public GyroResetCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.zeroHeading();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}