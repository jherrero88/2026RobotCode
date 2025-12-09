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
        AutoTrajectory trajectory0 = routine.trajectory("Test", 0);
        AutoTrajectory trajectory1 = routine.trajectory("Test", 1);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
            .andThen(trajectory1.cmd())
        );

        return routine;
    }

    // ————— competition routines ————— //

    public Command backUp() {
        return new DriveWithPosition(drive, new Transform2d(-2, 0, new Rotation2d()));
    }
}