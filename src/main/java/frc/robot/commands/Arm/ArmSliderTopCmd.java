package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSliderTopCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmSliderTopCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(armSubsystem.sliderEncoder.getPosition() > -Constants.ArmConstants.gArmSliderTop){
            armSubsystem.leftArmSlider.set(Constants.ArmConstants.gSliderSpeed);
            armSubsystem.rightArmSlider.set(-Constants.ArmConstants.gSliderSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // System.out.println(armSubsystem.sliderEncoder.getPosition() <= -Constants.ArmConstants.gArmSliderTop + Constants.ArmConstants.gArmOffset);
        // System.out.println(armSubsystem.sliderEncoder.getPosition());
        if(armSubsystem.sliderEncoder.getPosition() <= -Constants.ArmConstants.gArmSliderTop){
            return true;
        } else{
            return false;
        }
    }
}