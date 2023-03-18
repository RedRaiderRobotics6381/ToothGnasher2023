package frc.robot.commands.Auto.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.RotateSubsystem;

public class AutoManipulatorDriveCmd extends CommandBase {

    private final RotateSubsystem rotateSubsystem;
    double P;

    public AutoManipulatorDriveCmd(RotateSubsystem rotateSubsystem) {
        this.rotateSubsystem = rotateSubsystem;
        addRequirements(rotateSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        P = ((Math.abs(rotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.pos3) + 50) / 300);
        if (rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.pos3 + ArmConstants.rotateoffset) {
            rotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P);
            // System.out.println("up");
        }
        if (rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.pos3 - ArmConstants.rotateoffset) {
            rotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P);
            // System.out.println("down");
        }
    }

    @Override
    public void end(boolean interrupted) {
        rotateSubsystem.armRotateMotor.set(ArmConstants.pos3Gravity);
    }

    @Override
    public boolean isFinished() {
        if(rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.pos3 - ArmConstants.rotateoffset && rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.pos3 + ArmConstants.rotateoffset){
            return true;
        }else{
            return false;
        }
    }
}