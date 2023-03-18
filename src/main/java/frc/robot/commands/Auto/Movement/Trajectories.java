package frc.robot.commands.Auto.Movement;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Trajectories {

        public static Trajectory traj;

        public static SwerveControllerCommand traj1() {
                // 1. Define PID Controllers
                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // 2. Create trajectory settings
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4,
                                Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

                config.setReversed(true);

                // 3. Generate trajectory
                var start = new Pose2d(0, 0,
                                Rotation2d.fromDegrees(-180));
                var end = new Pose2d(2, 0,
                                Rotation2d.fromDegrees(-160));

                var interior = new ArrayList<Translation2d>();
                // interior.add(new Translation2d(Units.feetToMeters(14.54),
                // Units.feetToMeters(23.23)));
                // interior.add(new Translation2d(Units.feetToMeters(21.04),
                // Units.feetToMeters(18.23)));

                var trajectory = TrajectoryGenerator.generateTrajectory(
                                start,
                                interior,
                                end,
                                config);

                traj = trajectory;

                // 4. Construct command to follow trajectory
                SwerveControllerCommand swerveControllerCommandB = new SwerveControllerCommand(
                                trajectory,
                                RobotContainer.swerveSubsystem::getPose,
                                DriveConstants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                                RobotContainer.swerveSubsystem::setModuleStates,
                                RobotContainer.swerveSubsystem);

                return swerveControllerCommandB;
        }

        public static Trajectory getTraj1() {
                return traj;
        }

        public static SwerveControllerCommand traj2() {
                // 1. Define PID Controllers
                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // 2. Create trajectory settings
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4,
                                Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

                config.setReversed(true);

                // 3. Generate trajectory
                var start = new Pose2d(0, 0,
                                Rotation2d.fromDegrees(-180));
                var end = new Pose2d(2, 0,
                                Rotation2d.fromDegrees(-160));

                var interior = new ArrayList<Translation2d>();
                // interior.add(new Translation2d(Units.feetToMeters(14.54),
                // Units.feetToMeters(23.23)));
                // interior.add(new Translation2d(Units.feetToMeters(21.04),
                // Units.feetToMeters(18.23)));

                var trajectory = TrajectoryGenerator.generateTrajectory(
                                start,
                                interior,
                                end,
                                config);

                traj = trajectory;

                // 4. Construct command to follow trajectory
                SwerveControllerCommand swerveControllerCommandB = new SwerveControllerCommand(
                                trajectory,
                                RobotContainer.swerveSubsystem::getPose,
                                DriveConstants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                                RobotContainer.swerveSubsystem::setModuleStates,
                                RobotContainer.swerveSubsystem);

                return swerveControllerCommandB;
        }
}