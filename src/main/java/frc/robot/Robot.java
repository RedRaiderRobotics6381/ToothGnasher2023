// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;


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

    UsbCamera camera1;
    double targetPos = 150;

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

        camera1 = CameraServer.startAutomaticCapture(0);
        camera1.setResolution(160, 120);

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
        if(RobotContainer.armSubsystem.sliderEncoder.getPosition() < -42){
            RobotContainer.armSubsystem.leftArmSlider.set(0);
            RobotContainer.armSubsystem.rightArmSlider.set(0);
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
        RobotContainer.armSubsystem.wristRotateEncoder.setPosition(0);
        RobotContainer.armSubsystem.sliderEncoder.setPosition(0);

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

    //     double P = ((Math.abs(RobotContainer.armSubsystem.armRotateEncoder.getPosition() - targetPos)+50)/300);
    //     if(targetPos == ArmConstants.pos4){
    //         P = ((Math.abs(RobotContainer.armSubsystem.armRotateEncoder.getPosition() - targetPos)+30)/300);
    //     }else{
    //         P = ((Math.abs(RobotContainer.armSubsystem.armRotateEncoder.getPosition() - targetPos)+50)/300);
    //     }
    // //     System.out.println(RobotContainer.armSubsystem.armRotateEncoder.getPosition());

    //    if(RobotContainer.secondaryJoystick.getPOV() == 0){
    //     targetPos = ArmConstants.pos1;
    //     System.out.println("pos1");
    //    }
    //    if(RobotContainer.secondaryJoystick.getPOV() == 90){
    //     targetPos = ArmConstants.pos2;
    //     System.out.println("pos2");
    //    }
    //    if(RobotContainer.secondaryJoystick.getPOV() == 180){
    //     targetPos = ArmConstants.pos3;
    //     System.out.println("pos3");
    //    }
    //    if(RobotContainer.secondaryJoystick.getPOV() == 270){
    //     targetPos = ArmConstants.pos4;
    //     System.out.println("pos4");
    //    }
    //    RobotContainer.armSubsystem.armRotateMotor.set(0);
    //    if(RobotContainer.armSubsystem.armRotateEncoder.getPosition() > targetPos + ArmConstants.rotateoffset){
    //     RobotContainer.armSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P);
    //     // System.out.println("up");
    //    }else{
    //     if(targetPos == ArmConstants.pos1){
    //         RobotContainer.armSubsystem.armRotateMotor.set(ArmConstants.pos1Gravity);
    //     }
    //     if(targetPos == ArmConstants.pos2){
    //         RobotContainer.armSubsystem.armRotateMotor.set(ArmConstants.pos2Gravity);
    //     }
    //     if(targetPos == ArmConstants.pos3){
    //         RobotContainer.armSubsystem.armRotateMotor.set(ArmConstants.pos3Gravity);
    //     }
    //     if(targetPos == ArmConstants.pos4){
    //         RobotContainer.armSubsystem.armRotateMotor.set(ArmConstants.pos4Gravity);
    //     }
    //    }
    //    if(RobotContainer.armSubsystem.armRotateEncoder.getPosition() < targetPos - ArmConstants.rotateoffset){
    //     RobotContainer.armSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P);
    //     // System.out.println("down");
    //    }

    //    if(RobotContainer.secondaryJoystick.getRawAxis(5) < -0.25 && RobotContainer.armSubsystem.armRotateEncoder.getPosition() > ArmConstants.restriction1){
    //     RobotContainer.armSubsystem.armRotateMotor.set(RobotContainer.secondaryJoystick.getRawAxis(5)* 0.4);
    //     targetPos = RobotContainer.armSubsystem.armRotateEncoder.getPosition();
    //    }

    //    if(RobotContainer.secondaryJoystick.getRawAxis(5) > 0.25 && RobotContainer.armSubsystem.armRotateEncoder.getPosition() < ArmConstants.restriction2){
    //     RobotContainer.armSubsystem.armRotateMotor.set(RobotContainer.secondaryJoystick.getRawAxis(5)* 0.4);
    //     targetPos = RobotContainer.armSubsystem.armRotateEncoder.getPosition();
    //    }
    //     System.out.println(RobotContainer.armSubsystem.armRotateEncoder.getPosition());

        // RobotContainer.armSubsystem.armRotateMotor.set(RobotContainer.secondaryJoystick.getRawAxis(5) * 0.20);
        // }

        // System.out.println(ArmSubsystem.sliderEncoder.getPosition() + " Manual");

        // System.out.println(ArmSubsystem.wristRotateEncoder.getPosition());

        // System.out.println(ArmSubsystem.armRotateEncoder.getPosition());

        if(RobotContainer.armSubsystem.sliderEncoder.getPosition() < -42 && RobotContainer.secondaryJoystick.getRawAxis(1) > 0.25){
            RobotContainer.armSubsystem.leftArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * -Constants.ArmConstants.gSliderSpeed);
            RobotContainer.armSubsystem.rightArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * Constants.ArmConstants.gSliderSpeed);
        }
        else if(RobotContainer.armSubsystem.sliderEncoder.getPosition() > -0.5 && RobotContainer.secondaryJoystick.getRawAxis(1) < -0.25){
            RobotContainer.armSubsystem.leftArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * -Constants.ArmConstants.gSliderSpeed);
            RobotContainer.armSubsystem.rightArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * Constants.ArmConstants.gSliderSpeed);
        }
        else if(RobotContainer.armSubsystem.sliderEncoder.getPosition() < -42 || RobotContainer.armSubsystem.sliderEncoder.getPosition() > -0.5){
            RobotContainer.armSubsystem.leftArmSlider.set(0);
            RobotContainer.armSubsystem.rightArmSlider.set(0);
        }
        else{
            RobotContainer.armSubsystem.leftArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * -Constants.ArmConstants.gSliderSpeed);
            RobotContainer.armSubsystem.rightArmSlider.set(RobotContainer.secondaryJoystick.getRawAxis(1) * Constants.ArmConstants.gSliderSpeed);
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