package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        start = armSubsystem.armRotateEncoder.getPosition();
    }

    @Override
    public void execute() {
        if(armSubsystem.armRotateEncoder.getPosition() < change){
            armSubsystem.armRotateMotor.set(Constants.ArmConstants.gRotateSpeed);
        }
        if(armSubsystem.armRotateEncoder.getPosition() > change){
            armSubsystem.armRotateMotor.set(-Constants.ArmConstants.gRotateSpeed); // Might change negative
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.armRotateMotor.set(0.03);
    }

    @Override
    public boolean isFinished() {
        System.out.println(armSubsystem.armRotateEncoder.getPosition());

        if(armSubsystem.armRotateEncoder.getPosition() < change + Constants.ArmConstants.gRotateoffset && armSubsystem.armRotateEncoder.getPosition() > change - Constants.ArmConstants.gRotateoffset){
            return true;
        } else{
            return false;
        }
    }
}