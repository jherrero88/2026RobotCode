package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.*;

public class DriveWithVelocity extends Command {
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;
    
    private static final double DEADBAND = 0.1;

    public DriveWithVelocity(
        Drive drive, 
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier omegaSupplier
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void execute() {
        // Get linear velocity
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        // Apply rotation deadband
        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

        // Square rotation value for more precise control
        omega = Math.copySign(omega * omega, omega);

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec());
        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }
}