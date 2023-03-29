package frc.robot.commands.Drive.Allign;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.Primary.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveAllignPoleCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    Supplier<Boolean> button;
    static ChassisSpeeds chassisSpeeds;

    public DriveAllignPoleCmd(SwerveSubsystem swerveSubsystem, Supplier<Boolean> button) {
        this.button = button;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    @Override
    public void execute() {
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
            // NetworkTableInstance.get
            swerveSubsystem.setModuleStates(move());
        }
    }

    @Override
    public void end(boolean interrupted) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        // armSubsystem.intakeMotor.set(-0.05);
    }

    @Override
    public boolean isFinished() {
        if(button.get()){
            return false;
        } else{
            return true;
        }
    }

    public static SwerveModuleState[] move(){
        double speed = SensorConstants.PIDspeed.calculate(3.5, getDistance(getVerticle()));
        double side = -SensorConstants.PIDside.calculate(0, getHorizontal());
        double turn = -SensorConstants.PIDturn.calculate(0, getHorizontal());

        System.out.println("Side : " + side); // 0.3
        System.out.println("Turn : " + turn); // 0.03

        if(Math.abs(side) < 0.3 && Math.abs(turn) < 0.03){
            chassisSpeeds = new ChassisSpeeds(speed * 2.5, side, turn);
        } else{
            chassisSpeeds = new ChassisSpeeds(0, side, turn);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        return moduleStates;
    }

    public static double getDistance(double verticle) {
        double bottom = Math.tan(verticle * Math.PI / 180);
        double total = 2.83 / bottom; // originaly 3
        return total;
    }

    public static double getVerticle(){
        double verticle1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double reading = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        if (reading == 1) {
            return verticle1 + 45; // mount angle
        } else {
            return 0;
        }
    }

    public static double getHorizontal(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public static ChassisSpeeds stop() {
        return new ChassisSpeeds(0, 0, 0);
    }
}