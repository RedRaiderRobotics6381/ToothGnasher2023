// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmSliderBottomCmd;
import frc.robot.commands.Arm.ArmSliderHumanPlayerCmd;
import frc.robot.commands.Arm.ArmSliderTopCmd;
import frc.robot.commands.Arm.ArmWristCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if(ArmSubsystem.sliderEncoder.getPosition() < -42){
            ArmSubsystem.leftArmSlider.set(0);
            ArmSubsystem.rightArmSlider.set(0);
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        ArmSubsystem.wristRotateEncoder.setPosition(0);
        ArmSubsystem.sliderEncoder.setPosition(0);

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        //temporary!!!!!!
        // double rotateoffset = 0.0005;
        // double rotateSpeed = 0.3;
        // double positionPlace = 2.1;
        // double positionGrab = 1.1;

        System.out.println(ArmSubsystem.armRotateEncoder.getPosition());
        // ArmSubsystem.armRotateMotor.set(RobotContainer.secondaryJoystick.getRawAxis(5));
        // if (RobotContainer.secondaryJoystick.getRawButton(7)){
        //     if(ArmSubsystem.armRotateEncoder.getPosition() < positionGrab - rotateoffset){
        //         ArmSubsystem.armRotateMotor.set(rotateSpeed);
        //     }
        //     if(ArmSubsystem.armRotateEncoder.getPosition() > positionGrab + rotateoffset){
        //         // System.out.println("Woo");
        //         ArmSubsystem.armRotateMotor.set(rotateSpeed);
        //     }
        // }
        // if (RobotContainer.secondaryJoystick.getRawButton(8)){
        //     if(ArmSubsystem.armRotateEncoder.getPosition() < positionPlace - rotateoffset){
        //         ArmSubsystem.armRotateMotor.set(rotateSpeed);
        //     }
        //     if(ArmSubsystem.armRotateEncoder.getPosition() > positionPlace + rotateoffset){
        //         ArmSubsystem.armRotateMotor.set(-rotateSpeed);
        //     }
        // }

        // if(RobotContainer.secondaryJoystick.getRawAxis(5) * 0.20 < 0.03){
            // ArmSubsystem.armRotateMotor.set(0.03);
        // } else{
            ArmSubsystem.armRotateMotor.set(RobotContainer.secondaryJoystick.getRawAxis(5) * 0.20);
        // }

        // System.out.println(ArmSubsystem.sliderEncoder.getPosition() + " Manual");

        // System.out.println(ArmSubsystem.wristRotateEncoder.getPosition());

        // System.out.println(ArmSubsystem.armRotateEncoder.getPosition());
        System.out.println(RobotContainer.swerveSubsystem.getPitch() + " Pitch");

        if(ArmSubsystem.sliderEncoder.getPosition() < -42 && RobotContainer.secondaryJoystick.getRawAxis(1) > 0.25){
            ArmSubsystem.leftArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * -Constants.ArmConstants.gSliderSpeed);
            ArmSubsystem.rightArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * Constants.ArmConstants.gSliderSpeed);
        }
        else if(ArmSubsystem.sliderEncoder.getPosition() > -0.5 && RobotContainer.secondaryJoystick.getRawAxis(1) < -0.25){
            ArmSubsystem.leftArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * -Constants.ArmConstants.gSliderSpeed);
            ArmSubsystem.rightArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * Constants.ArmConstants.gSliderSpeed);
        }
        else if(ArmSubsystem.sliderEncoder.getPosition() < -42 || ArmSubsystem.sliderEncoder.getPosition() > -0.5){
            ArmSubsystem.leftArmSlider.set(0);
            ArmSubsystem.rightArmSlider.set(0);
        }
        else{
            ArmSubsystem.leftArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * -Constants.ArmConstants.gSliderSpeed);
            ArmSubsystem.rightArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * Constants.ArmConstants.gSliderSpeed);
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
