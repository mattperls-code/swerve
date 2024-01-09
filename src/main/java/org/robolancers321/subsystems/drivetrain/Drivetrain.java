package org.robolancers321.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
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

    private static double kTrackWidthMeters = Units.inchesToMeters(17.5);
    private static double kWheelBaseMeters = Units.inchesToMeters(17.5);

    private static double kMaxSpeedMetersPerSecond = 4.0;
    private static double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

    private static SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters), // front left
        new Translation2d(0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters), // front right
        new Translation2d(-0.5 * kTrackWidthMeters, 0.5 * kWheelBaseMeters), // back left
        new Translation2d(-0.5 * kTrackWidthMeters, -0.5 * kWheelBaseMeters) // back right
    );

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

        this.odometry = new SwerveDrivePoseEstimator(kSwerveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());

        // SwerveModule.initTuning();
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

        updateModules(states);
    }

    // for tuning
    public void drive(SwerveModuleState[] states){
        updateModules(states);
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
        // this.odometry.update(this.gyro.getRotation2d(), this.getModulePositions());

        this.doSendables();

        this.frontLeft.doSendables();
        this.frontRight.doSendables();
        this.backLeft.doSendables();
        this.backRight.doSendables();

        // this.frontLeft.tune();
        // this.frontRight.tune();
        // this.backLeft.tune();
        // this.backRight.tune();
    }
}
