package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    boolean autoVelocity;

    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);
    }

    // ————— raw command factories ————— //

    public Command getSetIntakeVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setIntakeVoltage(volts));
    }

    public Command getSetIntakeVelocityCommand(AngularVelocity velocity) { // ! idk if this is necessary
        return runOnce(() -> io.setIntakeVelocity(velocity));
    }

    public Command getSetPivotVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setPivotVoltage(volts));
    }

    // ————— processed command factories ————— //

    public Command getSetPivotUpCommand(Voltage volts) {
        return getSetPivotVoltageCommand(volts)
            .andThen(Commands.waitUntil(() -> io.getPivotUp()))
            .andThen(getSetPivotVoltageCommand(Volts.of(0)));
    }

    public Command getSetPivotDownCommand(Voltage volts) {
        return getSetPivotVoltageCommand(volts)
            .andThen(Commands.waitUntil(() -> io.getPivotDown()))
            .andThen(getSetPivotVoltageCommand(Volts.of(0)));
    }
}