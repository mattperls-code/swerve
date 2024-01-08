package org.robolancers321.commands.drivetrain;

import java.util.function.DoubleSupplier;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToAprilTag extends CommandBase {
    /*
     * Constants
     */

    private static double kTranslationP = 0.0;
    private static double kTranslationI = 0.0;
    private static double kTranslationD = 0.0;

    private static double kThetaP = 0.0;
    private static double kThetaI = 0.0;
    private static double kThetaD = 0.0;

    /*
     * Implementation
     */

    Drivetrain drivetrain;
    // PhotonCamera camera;

    DoubleSupplier desiredDxSupplier;
    DoubleSupplier desiredDzSupplier;
    DoubleSupplier desiredDThetaSupplier;

    PIDController xController;
    PIDController zController;
    PIDController thetaController;


    public MoveToAprilTag(DoubleSupplier desiredDxSupplier, DoubleSupplier desiredDzSupplier, DoubleSupplier desiredDThetaSupplier){
        this.drivetrain = Drivetrain.getInstance();
        // this.camera = new PhotonCamera("photonvision");

        this.desiredDxSupplier = desiredDxSupplier;
        this.desiredDzSupplier = desiredDzSupplier;
        this.desiredDThetaSupplier = desiredDThetaSupplier;

        this.xController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
        this.zController = new PIDController(kTranslationP, kTranslationI, kTranslationD);
        this.thetaController = new PIDController(kThetaP, kThetaI, kThetaD);
    }

    @Override
    public void execute(){
        double desiredDx = this.desiredDxSupplier.getAsDouble();
        double desiredDz = this.desiredDzSupplier.getAsDouble();
        double desiredDTheta = this.desiredDThetaSupplier.getAsDouble();

        this.xController.setSetpoint(desiredDx);
        this.zController.setSetpoint(desiredDz);
        this.thetaController.setSetpoint(desiredDTheta);
    }
}
