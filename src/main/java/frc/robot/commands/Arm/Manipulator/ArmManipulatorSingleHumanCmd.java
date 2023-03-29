package frc.robot.commands.Arm.Manipulator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Secondary.RotateSubsystem;

public class ArmManipulatorSingleHumanCmd extends CommandBase {

    private final RotateSubsystem rotateSubsystem;
    double P;

    public ArmManipulatorSingleHumanCmd(RotateSubsystem rotateSubsystem) {
        this.rotateSubsystem = rotateSubsystem;
        addRequirements(rotateSubsystem);
    }

    @Override
    public void initialize() {
        Constants.ArmConstants.manipulatorOn = true;
        ArmConstants.manipulatorManual = false;
    }

    @Override
    public void execute() {
        P = ((Math.abs(rotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.posSingularHuman)+30)/300);
        if(rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posSingularHuman + ArmConstants.rotateoffset){
            rotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P);
            // System.out.println("up");
           }
           if(rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posSingularHuman - ArmConstants.rotateoffset){
            rotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P);
            // System.out.println("down");
           }
    }

    @Override
    public void end(boolean interrupted) {
        rotateSubsystem.armRotateMotor.set(ArmConstants.posSingularHumanGravity);

        Constants.ArmConstants.manipulatorOn = false;
    }

    @Override
    public boolean isFinished() {
        if(rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posSingularHuman - ArmConstants.rotateoffset && rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posSingularHuman + ArmConstants.rotateoffset){
            return true;
        }else{
            return false;
        }
    }
}