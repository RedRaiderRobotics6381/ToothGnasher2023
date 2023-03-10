package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AutoOpenGrabberCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    Double change, start;

    public AutoOpenGrabberCmd(ArmSubsystem armSubsystem, Double change) {
        this.armSubsystem = armSubsystem;
        this.change = change;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.grabberEncoder.setPosition(0);
    }

    @Override
    public void execute() {
        armSubsystem.grabberMotor.set(-0.5);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.grabberMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println(armSubsystem.grabberEncoder.getPosition());
        if(armSubsystem.grabberEncoder.getPosition() <= -change){
            return true;
        } else{
            return false;
        }
    }
}