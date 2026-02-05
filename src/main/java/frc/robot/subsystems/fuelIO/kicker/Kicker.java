package frc.robot.subsystems.fuelIO.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("kicker", inputs);
    }

    // ————— raw command factories ————— //

    public Command getSetKickerVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setVoltage(volts));
    }
}