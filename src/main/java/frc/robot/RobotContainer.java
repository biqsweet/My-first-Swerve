// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.Controller;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.*;

public class RobotContainer {
    public static final Swerve SWERVE = new Swerve();
    private final Controller swerveController = new Controller(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        DoubleSupplier forwardSpeed = () -> MathUtil.applyDeadband(-swerveController.getRawAxis(LEFT_Y), 0.05);
        DoubleSupplier strafeSpeed = () -> MathUtil.applyDeadband(-swerveController.getRawAxis(LEFT_X), 0.05);
        DoubleSupplier turningSpeed = () -> MathUtil.applyDeadband(-swerveController.getRawAxis(RIGHT_X), 0.05);

        /*
        forwardSpeed,
        strafeSpeed,
        turningSpeed
        double yDeadbandValue = MathUtil.applyDeadband(forwardSpeed.getAsDouble(),0.05);
        double xDeadbandValue = MathUtil.applyDeadband(strafeSpeed.getAsDouble(),0.05);
        double rotationDeadbandValue = MathUtil.applyDeadband(turningSpeed.getAsDouble(),0.05);
         */

        SWERVE.setDefaultCommand(
                SWERVE.driveSelfRelative(
                        forwardSpeed,
                        strafeSpeed,
                        turningSpeed
                )
        );
    }
}
