package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

public class ArmWristCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    boolean clockwise = true;

    public ArmWristCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(clockwise == true){
            armSubsystem.wristRotateMotor.set(0.5);
        } else{
            armSubsystem.wristRotateMotor.set(-0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.wristRotateMotor.set(0);
        if(clockwise == true){
            clockwise = false;
        } else {
            clockwise = true;
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println(ArmSubsystem.wristRotateEncoder.getPosition());
        if(clockwise == true){
            if(Math.abs(ArmSubsystem.wristRotateEncoder.getPosition()) < 60){
                return false;
            } else{
                return true;
            }
        } else {
            if(ArmSubsystem.wristRotateEncoder.getPosition() <= 0){
                return true;
            } else{
                return false;
            }
        }
    }
}