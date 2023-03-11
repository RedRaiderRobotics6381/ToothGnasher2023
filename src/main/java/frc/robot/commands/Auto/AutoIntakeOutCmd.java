package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoIntakeOutCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    int time;

    public AutoIntakeOutCmd(ArmSubsystem armSubsystem, int time) {
        this.armSubsystem = armSubsystem;
        this.time = time;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        armSubsystem.intakeMotor.set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeMotor.set(-0.05);
    }

    @Override
    public boolean isFinished() {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        return true;
    }
}