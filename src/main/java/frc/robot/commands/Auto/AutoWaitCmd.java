package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoWaitCmd extends CommandBase {

    int goal;
    Boolean finished;

    public AutoWaitCmd(int goal) {
        this.goal = goal;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        try {
            Thread.sleep(goal);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        return true;
    }
}