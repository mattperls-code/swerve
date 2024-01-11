package org.robolancers321.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedForwardDrive extends Command {
    private Drivetrain drivetrain;

    private DoubleSupplier throttleSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier omegaSupplier;
    private BooleanSupplier fieldCentricSupplier;

    public FeedForwardDrive(DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier omegaSupplier, BooleanSupplier fieldCentricSupplier){
        this.drivetrain = Drivetrain.getInstance();

        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.omegaSupplier = omegaSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;

        this.addRequirements(this.drivetrain);
    }

    public FeedForwardDrive(double throttle, double strafe, double omega, boolean fieldCentric){
        this(() -> throttle, () -> strafe, () -> omega, () -> fieldCentric);
    }

    @Override
    public void execute(){
        double desiredThrottle = this.throttleSupplier.getAsDouble();
        double desiredStrafe = this.strafeSupplier.getAsDouble();
        double desiredOmega = this.omegaSupplier.getAsDouble();
        boolean desiredFieldCentic = this.fieldCentricSupplier.getAsBoolean();

        // TODO: any preprocessing for desired input should be done here
        desiredThrottle *= 1.2;
        desiredStrafe *= 1.2;
        desiredOmega *= -3.0;

        this.drivetrain.drive(desiredThrottle, desiredStrafe, desiredOmega, desiredFieldCentic);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        this.drivetrain.drive(0, 0, 0, false);
    }
}
