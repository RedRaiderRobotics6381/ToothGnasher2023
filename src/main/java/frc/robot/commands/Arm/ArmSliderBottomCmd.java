package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSliderBottomCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmSliderBottomCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(armSubsystem.sliderEncoder.getPosition() < -Constants.ArmConstants.gArmSliderBottom){
            armSubsystem.leftArmSlider.set(-Constants.ArmConstants.gSliderSpeed);
            armSubsystem.rightArmSlider.set(Constants.ArmConstants.gSliderSpeed); // maybe wrong here
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.leftArmSlider.set(0);
        armSubsystem.rightArmSlider.set(0);
    }

    @Override
    public boolean isFinished() {
        if(armSubsystem.sliderEncoder.getPosition() > -Constants.ArmConstants.gArmSliderBottom){
            return true;
        } else{
            return false;
        }
    }
}