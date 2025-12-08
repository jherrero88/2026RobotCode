package frc.robot.utils;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import choreo.auto.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;

public class AutoGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;

    public AutoGenerator(
        Drive drive, 
        SwerveDriveSimulation driveSimulation
    ) {
        autoFactory = new AutoFactory(
            drive::getPose,
            (pose) -> {
                drive.setPose(pose);
                driveSimulation.setSimulationWorldPose(pose);
            },
            drive::runAutoPosition,
            false, 
            // DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false,
            drive
        );

        this.drive = drive;
    }

    // ————— testing routines ————— //

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory = routine.trajectory("Test");

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory.resetOdometry()
            .andThen(trajectory.cmd())
            // .andThen(new DriveWithPosition(drive, trajectory.getFinalPose().get()))
        );

        return routine;
    }

    // ————— competition routines ————— //

    public Command backUp() {
        return new DriveWithPosition(drive, new Transform2d(-2, 0, new Rotation2d()));
    }
}