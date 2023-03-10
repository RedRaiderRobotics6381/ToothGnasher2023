package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AutoManipulatorCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    Double change, start;

    public AutoManipulatorCmd(ArmSubsystem armSubsystem, Double change) {
        this.armSubsystem = armSubsystem;
        this.change = change;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        start = ArmSubsystem.armRotateEncoder.getPosition();
    }

    @Override
    public void execute() {
        if(ArmSubsystem.armRotateEncoder.getPosition() < change){
            ArmSubsystem.armRotateMotor.set(Constants.ArmConstants.gRotateSpeed);
        }
        if(ArmSubsystem.armRotateEncoder.getPosition() > change){
            ArmSubsystem.armRotateMotor.set(-Constants.ArmConstants.gRotateSpeed); // Might change negative
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.armRotateMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println(ArmSubsystem.armRotateEncoder.getPosition());

        if(ArmSubsystem.armRotateEncoder.getPosition() < change + Constants.ArmConstants.gRotateoffset && ArmSubsystem.armRotateEncoder.getPosition() > change - Constants.ArmConstants.gRotateoffset){
            return true;
        } else{
            return false;
        }
    }
}