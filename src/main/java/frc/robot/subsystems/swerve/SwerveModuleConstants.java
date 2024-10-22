package frc.robot.subsystems.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderConfiguration;
import frc.lib.generic.hardware.encoder.EncoderSignal;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.generic.simulation.SimulationProperties;

import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.encoder.EncoderFactory.createCanCoder;
import static frc.lib.generic.hardware.motor.MotorFactory.createSpark;
import static frc.lib.generic.hardware.motor.MotorFactory.createTalonFX;

public class SwerveModuleConstants {
    private static final Motor DRIVE_MOTOR_FL = createTalonFX("driveMotorFL", 14);
    private static final Motor DRIVE_MOTOR_FR = createTalonFX("driveMotorFR", 3);
    private static final Motor DRIVE_MOTOR_BL = createTalonFX("driveMotorBL", 13);
    private static final Motor DRIVE_MOTOR_BR = createTalonFX("driveMotorBR", 2);

    private static final Motor TURNING_MOTOR_FL = createSpark("turningMotorFL", 11, MotorProperties.SparkType.MAX);
    private static final Motor TURNING_MOTOR_FR = createSpark("turningMotorFR", 10, MotorProperties.SparkType.MAX);
    private static final Motor TURNING_MOTOR_BL = createSpark("turningMotorBL", 6, MotorProperties.SparkType.MAX);
    private static final Motor TURNING_MOTOR_BR = createSpark("turningMotorBR", 9, MotorProperties.SparkType.MAX);

    private static final Encoder TURNING_ENCODER_FL = createCanCoder("turningEncoderFL", 18);
    private static final Encoder TURNING_ENCODER_FR = createCanCoder("turningEncoderFR", 20);
    private static final Encoder TURNING_ENCODER_BL = createCanCoder("turningEncoderBL", 19);
    private static final Encoder TURNING_ENCODER_BR = createCanCoder("turningEncoderBR", 21);

    private static final SwerveModule MODULE_FL = new SwerveModule(DRIVE_MOTOR_FL, TURNING_MOTOR_FL, TURNING_ENCODER_FL);
    private static final SwerveModule MODULE_FR = new SwerveModule(DRIVE_MOTOR_FR, TURNING_MOTOR_FR, TURNING_ENCODER_FR);
    private static final SwerveModule MODULE_BL = new SwerveModule(DRIVE_MOTOR_BL,TURNING_MOTOR_BL,TURNING_ENCODER_BL);
    private static final SwerveModule MODULE_BR = new SwerveModule(DRIVE_MOTOR_BR,TURNING_MOTOR_BR,TURNING_ENCODER_BR);

    private static final Motor[] MOTOR_LIST = {DRIVE_MOTOR_FL, DRIVE_MOTOR_FR, DRIVE_MOTOR_BL, DRIVE_MOTOR_BR};
    private static final Motor[] TURNING_MOTOR_LIST = {TURNING_MOTOR_FL, TURNING_MOTOR_FR, TURNING_MOTOR_BL, TURNING_MOTOR_BR};
    private static final Encoder[] ENCODER_LIST = {TURNING_ENCODER_FL, TURNING_ENCODER_FR, TURNING_ENCODER_BL, TURNING_ENCODER_BR};

    private static final MotorConfiguration TURNING_MOTOR_CONFIGURATION = new MotorConfiguration();
    private static final MotorConfiguration DRIVE_MOTOR_CONFIGURATION = new MotorConfiguration();
    private static final EncoderConfiguration ENCODER_CONFIGURATION = new EncoderConfiguration();

    static final SwerveModule[] MODULE_LIST = {MODULE_FL, MODULE_FR, MODULE_BL, MODULE_BR};
    static final double WHEEL_DIAMETER = Units.inchesToMeters(2);

    static {
        final double[] offsetList = {0.295850, 0.682871, 0.430674, 0.672080};

        configureMotorConfiguration();
        
        for (int i = 0; i < MODULE_LIST.length; i++) {
            driveMotorConfigurations(MOTOR_LIST[i]);
            turningMotorConfiguration(TURNING_MOTOR_LIST[i],ENCODER_LIST[i]);
            encoderConfiguration(ENCODER_LIST[i], offsetList[i]);
        }
    }

    private static void configureMotorConfiguration(){
        DRIVE_MOTOR_CONFIGURATION.gearRatio = 6.75;
        DRIVE_MOTOR_CONFIGURATION.supplyCurrentLimit = 70;
        DRIVE_MOTOR_CONFIGURATION.idleMode = MotorProperties.IdleMode.COAST;

        DRIVE_MOTOR_CONFIGURATION.slot0 = new MotorProperties.Slot(20, 0, 0, 0, 0, 0);

        DRIVE_MOTOR_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 6.75, 0.003);
        DRIVE_MOTOR_CONFIGURATION.simulationSlot = new MotorProperties.Slot(40, 0, 0, 0, 0, 0);

        TURNING_MOTOR_CONFIGURATION.gearRatio = 150.0 / 7;
        TURNING_MOTOR_CONFIGURATION.supplyCurrentLimit = 70;
        TURNING_MOTOR_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;
        TURNING_MOTOR_CONFIGURATION.inverted = true;
        TURNING_MOTOR_CONFIGURATION.closedLoopContinuousWrap = true;

        TURNING_MOTOR_CONFIGURATION.slot0 = new MotorProperties.Slot(20, 0, 0, 0, 0, 0);

        TURNING_MOTOR_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR, DCMotor.getFalcon500(1), 75 / 7.0, 0.003);
        TURNING_MOTOR_CONFIGURATION.simulationSlot = new MotorProperties.Slot(40, 0, 0, 0, 0, 0);
    }

    private static void driveMotorConfigurations(Motor driveMotor) {
        driveMotor.setupSignalUpdates(MotorSignal.VELOCITY);
        driveMotor.setupSignalUpdates(MotorSignal.VOLTAGE);
        driveMotor.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        driveMotor.configure(DRIVE_MOTOR_CONFIGURATION);
    }

    private static void turningMotorConfiguration(Motor turningMotor, Encoder turningEncoder) {
        turningMotor.setupSignalUpdates(MotorSignal.POSITION);
        turningMotor.setupSignalUpdates(MotorSignal.VOLTAGE);
        turningMotor.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        turningEncoder.setSimulatedEncoderPositionSource(turningMotor::getSystemPosition);
        turningEncoder.setSimulatedEncoderVelocitySource(turningMotor::getSystemVelocity);

        DoubleSupplier positionSupplier = turningEncoder::getEncoderPosition;
        turningMotor.setExternalPositionSupplier(positionSupplier);

        turningMotor.configure(TURNING_MOTOR_CONFIGURATION);
    }

    private static void encoderConfiguration(Encoder turningEncoder, double offset) {
        ENCODER_CONFIGURATION.offsetRotations = offset;
        turningEncoder.configure(ENCODER_CONFIGURATION);

        turningEncoder.setupSignalUpdates(EncoderSignal.POSITION);
    }
}
