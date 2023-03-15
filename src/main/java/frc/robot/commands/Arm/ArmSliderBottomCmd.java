package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSliderBottomCmd extends CommandBase {

    // private final ArmSubsystem armSubsystem;

    public ArmSliderBottomCmd(ArmSubsystem armSubsystem) {
        // this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(ArmSubsystem.sliderEncoder.getPosition() < -Constants.ArmConstants.gArmSliderBottom){
            ArmSubsystem.leftArmSlider.set(-Constants.ArmConstants.gSliderDown);
            ArmSubsystem.rightArmSlider.set(Constants.ArmConstants.gSliderDown);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.leftArmSlider.set(0);
        ArmSubsystem.rightArmSlider.set(0);
    }

    @Override
    public boolean isFinished() {
        if(ArmSubsystem.sliderEncoder.getPosition() > -Constants.ArmConstants.gArmSliderBottom){
            return true;
        } else{
            return false;
        }
    }
}