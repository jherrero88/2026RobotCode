package frc.robot.subsystems.fuelIO;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FuelConstants {
    // ————— CAN ids ————— //
    public static final int INTAKE_MOTOR_ID = 50;
    public static final int PIVOT_MOTOR_ID = 51;
    public static final int HOPPER_MOTOR_ID = 52;
    public static final int TURRET_MOTOR_ID = 53;
    public static final int ANGLER_MOTOR_ID = 54;
    public static final int SHOOTER_MOTOR_ID = 55;

    // ————— PIDF ————— //

    public static final ClosedLoopConfig INTAKE_PID = new ClosedLoopConfig().pid(0.3, 0, 0);
    public static final SimpleMotorFeedforward INTAKE_FF = new SimpleMotorFeedforward(0, 0.1, 0);

    public static final Slot0Configs SHOOTER_PIDF = new Slot0Configs()
    .withKP(0.3)
    .withKI(0)
    .withKD(0)
    .withKS(0)
    .withKV(0.1)
    .withKA(0);

    // the interpolated map should probably exist here
}