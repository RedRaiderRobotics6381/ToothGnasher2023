package frc.robot.commands.Auto.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Secondary.ArmSubsystem;

public class AutoIntakeOutCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public AutoIntakeOutCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        // this.time = time;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        armSubsystem.intakeMotor.set(-Constants.ArmConstants.gOutputSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeMotor.set(0.05);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}