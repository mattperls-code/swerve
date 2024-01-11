package org.robolancers321.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    /*
     * Singleton
     */

    private static Drivetrain instance = null;
    public static Drivetrain getInstance(){
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    /*
     * Constants
     */

    private static final double kTrackWidthMeters = Units.inchesToMeters(17.5);
    private static final double kWheelBaseMeters = Units.inchesToMeters(17.5);

    private static final double kMaxSpeedMetersPerSecond = 4.0;
    private static final double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

    private static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters), // front left
        new Translation2d(0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // front right
        new Translation2d(-0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters), // back left
        new Translation2d(-0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters) // back right
    );
    
    // TODO: are these constants different than those of the DriveToTarget command?
    private static final double kTranslationP = 0.0;
    private static final double kTranslationI = 0.0;
    private static final double kTranslationD = 0.0;

    private static final double kRotationP = 0.0;
    private static final double kRotationI = 0.0;
    private static final double kRotationD = 0.0;

    /*
     * Implementation
     */

    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;

    private AHRS gyro;

    private SwerveDrivePoseEstimator odometry;

    private Drivetrain(){
        this.frontLeft = SwerveModule.getFrontLeft();
        this.frontRight = SwerveModule.getFrontRight();
        this.backLeft = SwerveModule.getBackLeft();
        this.backRight = SwerveModule.getBackRight();

        this.gyro = new AHRS(SPI.Port.kMXP);
        this.zeroYaw();

        this.odometry = new SwerveDrivePoseEstimator(kSwerveKinematics, this.gyro.getRotation2d(), this.getModulePositions(), new Pose2d());

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(kTranslationP, kTranslationI, kTranslationD),
                new PIDConstants(kRotationP, kRotationI, kRotationD),
                kMaxSpeedMetersPerSecond,
                0.5 * Math.hypot(kTrackWidthMeters, kWheelBaseMeters),
                new ReplanningConfig()
            ),
            () -> {
                var myAlliance = DriverStation.getAlliance();

                if (myAlliance.isPresent()) return myAlliance.get() == DriverStation.Alliance.Red;

                return false;
            },
            this
        );
    }

    public Pose2d getPose(){
        return this.odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose){
        this.odometry.resetPosition(this.gyro.getRotation2d(), this.getModulePositions(), pose);
    }

    private SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    public ChassisSpeeds getChassisSpeeds(){
        return kSwerveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    private SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public double getYawDeg(){
        return this.gyro.getYaw();
    }

    public Pose2d getOdometryEstimatedPose(){
        return this.odometry.getEstimatedPosition();
    }

    public void zeroYaw(){
        this.gyro.zeroYaw();
    }

    public void updateModules(SwerveModuleState[] states){
        this.frontLeft.update(states[0]);
        this.frontRight.update(states[1]);
        this.backLeft.update(states[2]);
        this.backRight.update(states[3]);
    }

    public void drive(double desiredThrottleMPS, double desiredStrafeMPS, double desiredOmegaRadPerSec, boolean fieldRelative){
        double correctedOmega = MathUtil.clamp(desiredOmegaRadPerSec, -kMaxOmegaRadiansPerSecond, kMaxOmegaRadiansPerSecond);

        // apply corrective pose logarithm

        // TODO: dt is technically an empirical constant and may need to be adjusted
        double dt = 0.2;

        double angularDisplacement = correctedOmega * dt;

        double sin = Math.sin(0.5 * angularDisplacement);
        double cos = Math.cos(0.5 * angularDisplacement);

        double correctedThrottle = desiredStrafeMPS * sin + desiredThrottleMPS * cos;
        double correctedStrafe = desiredStrafeMPS * cos - desiredThrottleMPS * sin;

        ChassisSpeeds speeds = fieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(correctedStrafe, correctedThrottle, correctedOmega, this.gyro.getRotation2d()) :
            new ChassisSpeeds(correctedStrafe, correctedThrottle, correctedOmega);
        
        this.drive(speeds);
    }

    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] states = kSwerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

        this.updateModules(states);
    }

    public void tuneDrive(SwerveModuleState[] states){
        this.frontLeft.tune();
        this.frontRight.tune();
        this.backLeft.tune();
        this.backRight.tune();

        this.updateModules(states);
    }

    private void doSendables(){
        SmartDashboard.putNumber("Drive Heading", this.getYawDeg());

        Pose2d odometryEstimatedPose = this.getOdometryEstimatedPose();

        SmartDashboard.putNumber("Odometry Pos X (m)", odometryEstimatedPose.getX());
        SmartDashboard.putNumber("Odometry Pos Y (m)", odometryEstimatedPose.getY());

        // TODO: is this angle or omega?
        SmartDashboard.putNumber("Odometry Angle (deg)", odometryEstimatedPose.getRotation().getDegrees());
    }

    @Override
    public void periodic(){
        // TODO: this breaks with photon vision installed?
        this.odometry.update(this.gyro.getRotation2d(), this.getModulePositions());

        this.doSendables();

        this.frontLeft.doSendables();
        this.frontRight.doSendables();
        this.backLeft.doSendables();
        this.backRight.doSendables();
    }
}
