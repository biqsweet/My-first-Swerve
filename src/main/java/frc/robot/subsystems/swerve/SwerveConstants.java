package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveConstants {
    protected static final double WIDTH = 0.68;
    protected static final double LENGTH = 0.56;

    protected static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(LENGTH / 2, WIDTH / 2),
            new Translation2d(LENGTH / 2, -WIDTH / 2),
            new Translation2d(-LENGTH / 2, WIDTH / 2),
            new Translation2d(-LENGTH / 2, -WIDTH / 2)
    );
}