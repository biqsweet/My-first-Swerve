package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.lib.math.Conversions.rpsToMps;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.WHEEL_DIAMETER;

public class SwerveModule {
    private final Motor driveMotor;
    private final Motor turningMotor;
    private final Encoder turningEncoder;
    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(Motor driveMotor, Motor turningMotor, Encoder turningEncoder) {
        this.driveMotor = driveMotor;
        this.turningMotor = turningMotor;
        this.turningEncoder = turningEncoder;
    }

    public void stop() {
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    public SwerveModuleState getCurrentStates() {
//        Measure<Distance> distance = Meters.of(getCurrentVelocity()).times(Math.PI).times(WHEEL_DIAMETER.in(Meters));
        return new SwerveModuleState(rpsToMps(getCurrentVelocity(), WHEEL_DIAMETER), getCurrentAngle());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public void setTargetState(SwerveModuleState state) {
        final SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getCurrentAngle());
        setTargetVelocity(optimizedState.speedMetersPerSecond);
        setTargetAngle(optimizedState.angle);

//        Logger.recordOutput("speed", optimizedState.speedMetersPerSecond);
//        Logger.recordOutput("angle", optimizedState.angle);

//        setTargetVelocity(state.speedMetersPerSecond);
//        setTargetAngle(state.angle);

        targetState = optimizedState;
    }

    private void setTargetVelocity(double voltage) {
        driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    private void setTargetAngle(Rotation2d angle) {
        turningMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    private double getCurrentVelocity() {
        return driveMotor.getSystemVelocity();
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(turningEncoder.getEncoderPosition());
    }
}
