package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class AutoManipulatorCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    int position;
    double targetPosition;
    double rotateoffset = 5;
    double rotateSpeed = 0.5;
    double counterWeight = 0.1;

    public AutoManipulatorCmd(ArmSubsystem armSubsystem, int position) {
        this.armSubsystem = armSubsystem;
        this.position = position;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        targetPosition = position;
    }

    @Override
    public void execute() {
        double P = ((Math.abs(armSubsystem.armRotateEncoder.getPosition() - targetPosition)+50)/300);
       
       if(armSubsystem.armRotateEncoder.getPosition() > targetPosition + rotateoffset){
        RobotContainer.armSubsystem.armRotateMotor.set(-rotateSpeed * P);
       }
       if(armSubsystem.armRotateEncoder.getPosition() < targetPosition - rotateoffset){
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
        if(armSubsystem.armRotateEncoder.getPosition() <= targetPosition + rotateoffset && armSubsystem.armRotateEncoder.getPosition() >= targetPosition - rotateoffset){
            return true;
        } else{
            return false;
        }
    }
}