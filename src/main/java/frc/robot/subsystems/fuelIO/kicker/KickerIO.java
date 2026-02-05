package frc.robot.subsystems.fuelIO.kicker;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;

public interface KickerIO {
    @AutoLog
    class KickerIOInputs {
        public double kickerVoltage = 0.0;
        public double kickerCurrent = 0.0;
        public double kickerPosition = 0.0;
        public double kickerVelocity = 0.0;
        public double kickerTemperature = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setVoltage(Voltage volts) {}
}