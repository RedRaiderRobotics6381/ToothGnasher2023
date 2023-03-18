package frc.robot.commands.Arm.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmIntakeInCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    Supplier<Boolean> button;

    public ArmIntakeInCmd(ArmSubsystem armSubsystem, Supplier<Boolean> button) {
        this.armSubsystem = armSubsystem;
        this.button = button;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.secondaryJoystick.setRumble(RumbleType.kRightRumble, 0.5);
    }

    @Override
    public void execute() {
        armSubsystem.intakeMotor.set(Constants.ArmConstants.gIntakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeMotor.set(0.05);
        RobotContainer.secondaryJoystick.setRumble(RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        if(button.get() == true){
            return false;
        }
        else{
            return true;
        }
    }
}