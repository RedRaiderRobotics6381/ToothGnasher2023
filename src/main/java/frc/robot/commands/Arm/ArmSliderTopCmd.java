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
        if(ArmSubsystem.sliderEncoder.getPosition() > -Constants.ArmConstants.gArmSliderTop){
            ArmSubsystem.leftArmSlider.set(Constants.ArmConstants.gSliderSpeed);
            ArmSubsystem.rightArmSlider.set(-Constants.ArmConstants.gSliderSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.leftArmSlider.set(0);
        ArmSubsystem.rightArmSlider.set(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println(ArmSubsystem.sliderEncoder.getPosition() + " Top");
        if(ArmSubsystem.sliderEncoder.getPosition() <= -Constants.ArmConstants.gArmSliderTop){
            return true;
        } else{
            return false;
        }
    }
}