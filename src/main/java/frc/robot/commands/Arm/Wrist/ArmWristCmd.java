package frc.robot.commands.Arm.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Secondary.ArmSubsystem;

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
            armSubsystem.wristRotateMotor.set(0.9 * ((Math.abs(armSubsystem.wristRotateEncoder.getPosition()-90)+20)/180));
        } else{
            armSubsystem.wristRotateMotor.set(-0.9 * ((Math.abs(armSubsystem.wristRotateEncoder.getPosition()-270)+50)/180));
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
        // System.out.println(ArmSubsystem.wristRotateEncoder.getPosition());
        if(clockwise == true){
            if(armSubsystem.wristRotateEncoder.getPosition() > 90){
                return false;
            } else{
                return true;
            }
        } else {
            if(armSubsystem.wristRotateEncoder.getPosition() >= 250){
                return true;
            } else{
                return false;
            }
        }
    }
}