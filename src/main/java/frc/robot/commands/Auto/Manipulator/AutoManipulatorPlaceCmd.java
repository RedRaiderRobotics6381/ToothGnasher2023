package frc.robot.commands.Auto.Manipulator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RotateSubsystem;

public class AutoManipulatorPlaceCmd extends CommandBase {

    private final RotateSubsystem rotateSubsystem;
    double P;

    public AutoManipulatorPlaceCmd(RotateSubsystem rotateSubsystem) {
        this.rotateSubsystem = rotateSubsystem;
        addRequirements(rotateSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        P = ((Math.abs(armSubsystem.armRotateEncoder.getPosition() - ArmConstants.pos2)+50)/300);
        if(armSubsystem.armRotateEncoder.getPosition() > ArmConstants.pos2 + ArmConstants.rotateoffset){
            armSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P);
            // System.out.println("up");
           }
           if(armSubsystem.armRotateEncoder.getPosition() < ArmConstants.pos2 - ArmConstants.rotateoffset){
            armSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P);
            // System.out.println("down");
           }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.armRotateMotor.set(ArmConstants.pos2Gravity);
    }

    @Override
    public boolean isFinished() {
        if(armSubsystem.armRotateEncoder.getPosition() > ArmConstants.pos2 - ArmConstants.rotateoffset && armSubsystem.armRotateEncoder.getPosition() < ArmConstants.pos2 + ArmConstants.rotateoffset){
            return true;
        }else{
            return false;
        }
    }
}