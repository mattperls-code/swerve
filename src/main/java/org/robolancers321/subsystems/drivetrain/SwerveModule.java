package org.robolancers321.subsystems.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    /*
     * Singletons
     */

    private static SwerveModule frontLeft = null;
    public static SwerveModule getFrontLeft(){
        if (frontLeft == null) frontLeft = new SwerveModule("Front Left", 4, 3, 15, true, false, false, 415.546875);

        return frontLeft;
    }

    private static SwerveModule frontRight = null;
    public static SwerveModule getFrontRight(){
        if (frontRight == null) frontRight = new SwerveModule("Front Right", 6, 5, 16, false, false, false, -84.7265625);

        return frontRight;
    }

    private static SwerveModule backLeft = null;
    public static SwerveModule getBackLeft(){
        if (backLeft == null) backLeft = new SwerveModule("Back Left", 2, 1, 14, false, false, false, 298.65234375);

        return backLeft;
    }

    private static SwerveModule backRight = null;
    public static SwerveModule getBackRight(){
        if (backRight == null) backRight = new SwerveModule("Back Right", 8, 7, 13, true, false, false, 349.98046875);

        return backRight;
    }

    /*
     * Constants
     */

    private static CANCoderConfiguration kCANCoderConfig = new CANCoderConfiguration();

    static {
        kCANCoderConfig.sensorCoefficient = (2.0 * Math.PI) / (4096.0);
        kCANCoderConfig.unitString = "rad";
        kCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        kCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    }

    private static double kWheelRadiusMeters = Units.inchesToMeters(1.5);
    private static double kGearRatio = 6.8;
    private static double kRPMToMPS = 2 * Math.PI * kWheelRadiusMeters / (kGearRatio * 60.0);

    private static double kDriveP = 0.00;
    private static double kDriveI = 0.00;
    private static double kDriveD = 0.00;
    private static double kDriveFF = 0.20;

    private static double kTurnP = 0.50;
    private static double kTurnI = 0.00;
    private static double kTurnD = 0.00;

    /*
     * Implementation
     */

    private String id;

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private RelativeEncoder driveEncoder;
    private CANCoder turnEncoder;

    private SparkMaxPIDController driveController;
    private PIDController turnController;

    private SwerveModule(String id, int driveMotorPort, int turnMotorPort, int turnEncoderPort, boolean invertDriveMotor, boolean invertTurnMotor, boolean invertTurnEncoder, double turnEncoderOffset){
        this.id = id;

        this.configDrive(driveMotorPort, invertDriveMotor);
        this.configTurn(turnMotorPort, turnEncoderPort, invertTurnMotor, invertTurnEncoder, turnEncoderOffset);
    }

    private void configDrive(int driveMotorPort, boolean invertDriveMotor){
        this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);

        this.driveMotor.setInverted(invertDriveMotor);
        this.driveMotor.setIdleMode(IdleMode.kBrake);
        this.driveMotor.setSmartCurrentLimit(40);
        this.driveMotor.enableVoltageCompensation(12);
        this.driveMotor.burnFlash();

        this.driveEncoder = this.driveMotor.getEncoder();
        this.driveEncoder.setVelocityConversionFactor(kRPMToMPS);

        this.driveController = this.driveMotor.getPIDController();
        this.driveController.setP(kDriveP);
        this.driveController.setI(kDriveI);
        this.driveController.setD(kDriveD);
        this.driveController.setFF(kDriveFF);
    }

    private void configTurn(int turnMotorPort, int turnEncoderPort, boolean invertTurnMotor, boolean invertTurnEncoder, double turnEncoderOffset){
        this.turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
        
        this.turnMotor.setInverted(invertTurnMotor);
        this.turnMotor.setIdleMode(IdleMode.kBrake);
        this.turnMotor.setSmartCurrentLimit(40);
        this.turnMotor.enableVoltageCompensation(12);
        this.turnMotor.burnFlash();

        this.turnEncoder = new CANCoder(turnEncoderPort);

        CANCoderConfiguration config = kCANCoderConfig;
        config.magnetOffsetDegrees = turnEncoderOffset;
        config.sensorDirection = invertTurnEncoder;

        this.turnEncoder.configAllSettings(config);

        this.turnController = new PIDController(kTurnP, kTurnI, kTurnD);
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDriveVelocity(){
        return this.driveEncoder.getVelocity();
    }

    public double getTurnAngleRad(){
        return this.turnEncoder.getAbsolutePosition();
    }

    public double getTurnAngleDeg(){
        return this.getTurnAngleRad() * 180.0 / Math.PI;
    }

    public SwerveModulePosition getPosition(){
        // TODO: should drive encoder position have a conversion factor?
        return new SwerveModulePosition(this.driveEncoder.getPosition(), Rotation2d.fromRadians(this.getTurnAngleRad()));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(this.getDriveVelocity(), Rotation2d.fromRadians(this.getTurnAngleRad()));
    }

    public void update(SwerveModuleState desiredState){
        SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(this.getTurnAngleRad()));

        this.driveController.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);

        this.turnController.setSetpoint(optimized.angle.getRadians());
        this.turnMotor.set(-MathUtil.clamp(this.turnController.calculate(this.getTurnAngleRad()), -1.0, 1.0));
    }

    public void doSendables(){
        SmartDashboard.putNumber(this.id + " Drive Vel (m/s)", this.getDriveVelocity());
        SmartDashboard.putNumber(this.id + " Turn Angle (deg)", this.getTurnAngleDeg());
    }

    public static void initTuning(){
        SmartDashboard.putNumber("drive kp", SmartDashboard.getNumber("drive kp", kDriveP));
        SmartDashboard.putNumber("drive ki", SmartDashboard.getNumber("drive ki", kDriveI));
        SmartDashboard.putNumber("drive kd", SmartDashboard.getNumber("drive kd", kDriveD));
        SmartDashboard.putNumber("drive kff", SmartDashboard.getNumber("drive kff", kDriveFF));
        
        SmartDashboard.putNumber("turn kp", SmartDashboard.getNumber("turn kp", kTurnP));
        SmartDashboard.putNumber("turn ki", SmartDashboard.getNumber("turn ki", kTurnI));
        SmartDashboard.putNumber("turn kd", SmartDashboard.getNumber("turn kd", kTurnD));
    }

    public void tune(){
        double tunedDriveP = SmartDashboard.getNumber("drive kp", kDriveP);
        double tunedDriveI = SmartDashboard.getNumber("drive ki", kDriveI);
        double tunedDriveD = SmartDashboard.getNumber("drive kd", kDriveD);
        double tunedDriveFF = SmartDashboard.getNumber("drive kff", kDriveFF);

        this.driveController.setP(tunedDriveP);
        this.driveController.setI(tunedDriveI);
        this.driveController.setD(tunedDriveD);
        this.driveController.setFF(tunedDriveFF);

        double tunedTurnP = SmartDashboard.getNumber("turn kp", kTurnP);
        double tunedTurnI = SmartDashboard.getNumber("turn ki", kTurnI);
        double tunedTurnD = SmartDashboard.getNumber("turn kd", kTurnD);

        this.turnController.setPID(tunedTurnP, tunedTurnI, tunedTurnD);
    }
}
