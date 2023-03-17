package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AutoRotateCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    int position;
    double targetposition;
    double rotateoffset = 5;
    double position1 = 95;
    double position2 = 115;
    double position3 = 180;
    double position4 = 285;
    double rotateSpeed = 0.5;
    double counterWeight = 0.1;

    public AutoRotateCmd(ArmSubsystem armSubsystem, int position) {
        this.armSubsystem = armSubsystem;
        this.position = position;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if(position == 1){
            targetposition = position1;
        }
        if(position == 2){
            targetposition = position2;
        }
        if(position == 3){
            targetposition = position3;
        }
        if(position == 4){
            targetposition = position4;
        }
    }

    @Override
    public void execute() {
        double P = ((Math.abs(ArmSubsystem.armRotateEncoder.getPosition() - targetposition)+50)/300);
       
       if(ArmSubsystem.armRotateEncoder.getPosition() > targetposition + rotateoffset){
        RobotContainer.armSubsystem.armRotateMotor.set(-rotateSpeed * P);
       }
       if(ArmSubsystem.armRotateEncoder.getPosition() < targetposition - rotateoffset){
        RobotContainer.armSubsystem.armRotateMotor.set(rotateSpeed * P);
       }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.armSubsystem.armRotateMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println(armSubsystem.grabberEncoder.getPosition());
        if(armSubsystem.armRotateEncoder.getPosition() <= targetposition + rotateoffset && armSubsystem.armRotateEncoder.getPosition() >= targetposition - rotateoffset){
            return true;
        } else{
            return false;
        }
    }
}