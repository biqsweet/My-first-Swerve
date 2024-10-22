package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.generic.simulation.GyroSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;
import static frc.robot.subsystems.swerve.SwerveModuleConstants.MODULE_LIST;

public class Swerve extends SubsystemBase {
    public Command driveSelfRelative(DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier turningSpeed) {
        return Commands.run(() -> {
                    ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed.getAsDouble(), strafeSpeed.getAsDouble(), turningSpeed.getAsDouble());
                    final SwerveModuleState[] moduleStates = SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

                    for (int i = 0; i < MODULE_LIST.length; i++) {
                        MODULE_LIST[i].setTargetState(moduleStates[i]);
                    }
                }, this
        );
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getCurrentStates());
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            for (int i = 0; i < MODULE_LIST.length; i++) {
                MODULE_LIST[i].stop();
            }
        });
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getCurrentStates() {
        final SwerveModuleState[] currentStates = new SwerveModuleState[4];

        for (int i = 0; i < MODULE_LIST.length; i++) {
            currentStates[i] = MODULE_LIST[i].getCurrentStates();
        }
        return currentStates;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    private SwerveModuleState[] getTargetModuleStates() {
        final SwerveModuleState[] targetStates = new SwerveModuleState[4];

        for (int i = 0; i < MODULE_LIST.length; i++) {
            targetStates[i] = MODULE_LIST[i].getTargetState();
        }
        return targetStates;
    }
}