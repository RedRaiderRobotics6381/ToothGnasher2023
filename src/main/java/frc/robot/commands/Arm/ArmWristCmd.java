package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

public class ArmWristCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    public Supplier<Boolean> button;

    public ArmWristCmd(ArmSubsystem armSubsystem, Supplier<Boolean> button) {
        this.button = button;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.wristRotateMotor.set(-0.5);
        System.out.println(armSubsystem.wristRotateEncoder.getPosition());
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.wristRotateMotor.set(0);
        armSubsystem.wristRotateEncoder.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        // if(button.get()){
        if(armSubsystem.wristRotateEncoder.getPosition() < Math.abs(60)){
            return false;
        } else{
            return true;
        }
    }
}