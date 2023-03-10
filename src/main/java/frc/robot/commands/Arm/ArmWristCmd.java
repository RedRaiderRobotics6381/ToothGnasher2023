package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

public class ArmWristCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    public Supplier<Double> movement;

    public ArmWristCmd(ArmSubsystem armSubsystem) {
        this.movement = movement;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.wristRotateMotor.set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.wristRotateMotor.set(0);
        armSubsystem.wristRotateEncoder.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        if(armSubsystem.wristRotateEncoder.getPosition() < 10){
            return false;
        } else{
            return true;
        }
    }
}