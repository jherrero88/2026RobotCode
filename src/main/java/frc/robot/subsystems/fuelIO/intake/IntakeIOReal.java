package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax intakeMotor;
    private final SparkMax pivotMotor;

    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private final SimpleMotorFeedforward intakeFeedforward;

    private final RelativeEncoder intakeEncoder;
    private final RelativeEncoder pivotEncoder;

    private AngularVelocity prevIntakeVelocity = RadiansPerSecond.of(0.0);

    private final Trigger upSwitch;
    private final Trigger downSwitch;

    @SuppressWarnings("resource")
    public IntakeIOReal(
        int intakeId, 
        int pivotId, 
        int upSwitchPort,
        int downSwitchPort
    ) {
        intakeMotor = new SparkMax(intakeId, MotorType.kBrushless);
        pivotMotor = new SparkMax(pivotId, MotorType.kBrushless);

        // start config
        intakeMotor.setCANTimeout(500);
        pivotMotor.setCANTimeout(500);

        // encoders
        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0.0);
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0.0);

        // pid 
        intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).apply(FuelConstants.INTAKE_PID);
        intakeFeedforward = FuelConstants.INTAKE_FF;
        
        // miscellaneous settings
        intakeConfig.signals.primaryEncoderVelocityPeriodMs(10);
        intakeConfig.encoder.quadratureMeasurementPeriod(10);
        intakeConfig.encoder.quadratureAverageDepth(2);
        
        intakeConfig.smartCurrentLimit(30);
        intakeConfig.voltageCompensation(12);
        pivotConfig.smartCurrentLimit(30);
        pivotConfig.voltageCompensation(12);

        // ! intakeConfig.inverted(true);
        // ! pivotConfig.inverted(true);

        intakeConfig.idleMode(IdleMode.kBrake);
        pivotConfig.idleMode(IdleMode.kBrake);

        // stop config
        intakeMotor.setCANTimeout(0);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.setCANTimeout(0);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        upSwitch = new Trigger(new DigitalInput(upSwitchPort)::get);
        downSwitch = new Trigger(new DigitalInput(downSwitchPort)::get);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeCurrent = intakeMotor.getOutputCurrent();
        inputs.intakePosition = intakeEncoder.getPosition();
        inputs.intakeVelocity = Rotations.per(Minute).of(intakeEncoder.getVelocity()).in(RotationsPerSecond);
        inputs.intakeTemperature = intakeMotor.getMotorTemperature();

        inputs.pivotVoltage = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotCurrent = pivotMotor.getOutputCurrent();
        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = Rotations.per(Minute).of(pivotEncoder.getVelocity()).in(RotationsPerSecond);
        inputs.pivotTemperature = pivotMotor.getMotorTemperature();
    }

    @Override
    public void setIntakeVoltage(Voltage volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setIntakeVelocity(AngularVelocity velocity) {
        intakeMotor.getClosedLoopController().setSetpoint(
            velocity.in(Rotations.per(Minute)), 
            SparkMax.ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0, 
            intakeFeedforward.calculateWithVelocities(
                prevIntakeVelocity.in(RadiansPerSecond),
                velocity.in(RadiansPerSecond)
            )
        );

        prevIntakeVelocity = velocity;
    }

    @Override
    public void setPivotVoltage(Voltage volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public boolean getPivotUp() {
        return upSwitch.getAsBoolean();
    }

    @Override
    public boolean getPivotDown() {
        return downSwitch.getAsBoolean();
    }
}