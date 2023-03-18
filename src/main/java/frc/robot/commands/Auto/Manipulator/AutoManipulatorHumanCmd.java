package frc.robot.commands.Auto.Manipulator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.RotateSubsystem;

public class AutoManipulatorHumanCmd extends CommandBase {

    private final RotateSubsystem rotateSubsystem;
    double P;

    public AutoManipulatorHumanCmd(RotateSubsystem rotateSubsystem) {
        this.rotateSubsystem = rotateSubsystem;
        addRequirements(rotateSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        P = ((Math.abs(rotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.pos1)+50)/300);
        if(rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.pos1 + ArmConstants.rotateoffset){
            rotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P);
            // System.out.println("up");
           }
           if(rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.pos1 - ArmConstants.rotateoffset){
            rotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P);
            // System.out.println("down");
           }
    }

    @Override
    public void end(boolean interrupted) {
        rotateSubsystem.armRotateMotor.set(ArmConstants.pos1Gravity);
    }

    @Override
    public boolean isFinished() {
        if(rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.pos1 - ArmConstants.rotateoffset && rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.pos1 + ArmConstants.rotateoffset){
            return true;
        }else{
            return false;
        }
    }
}