package org.robolancers321.commands.drivetrain;

import java.util.function.DoubleSupplier;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
    private Drivetrain drivetrain;

    private DoubleSupplier throttleSupplier;
    private DoubleSupplier strafeSupplier;
    private DoubleSupplier omegaSupplier;

    public TeleopDrive(DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier omegaSupplier){
        this.drivetrain = Drivetrain.getInstance();

        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.omegaSupplier = omegaSupplier;

        addRequirements(this.drivetrain);
    }

    @Override
    public void execute(){
        double desiredThrottle = this.throttleSupplier.getAsDouble();
        double desiredStrafe = this.strafeSupplier.getAsDouble();
        double desiredOmega = this.omegaSupplier.getAsDouble();

        // TODO: any preprocessing for desired input should be done here

        // for debug
        SmartDashboard.putNumber("drive throttle", desiredThrottle);
        SmartDashboard.putNumber("drive strafe", desiredStrafe);
        SmartDashboard.putNumber("drive omega", desiredOmega);

        this.drivetrain.drive(desiredThrottle, desiredStrafe, desiredOmega, true);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        this.drivetrain.drive(0, 0, 0, true);
    }
}
