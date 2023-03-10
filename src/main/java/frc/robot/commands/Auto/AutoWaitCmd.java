package frc.robot.commands.Auto;

import java.util.function.Supplier;

import java.util.Timer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AutoWaitCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    int goal;
    Boolean finished;

    public AutoWaitCmd(ArmSubsystem armSubsystem, int goal) {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
        addRequirements(armSubsystem);
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