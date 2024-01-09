package org.robolancers321.commands.drivetrain;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TuneDrive extends CommandBase {
    private Drivetrain drivetrain;

    public TuneDrive(){
        this.drivetrain = Drivetrain.getInstance();

        SmartDashboard.putNumber("target angle (deg)", 0.0);
        SmartDashboard.putNumber("target vel (m/s)", 0.0);

        addRequirements(this.drivetrain);
    }

    @Override
    public void execute(){
        double theta = SmartDashboard.getNumber("target angle (deg)", 0.0);
        double vel = SmartDashboard.getNumber("target vel (m/s)", 0.0);

        SwerveModuleState[] states = new SwerveModuleState[4];

        for(int i = 0;i<4;i++){
            states[i] = new SwerveModuleState(vel, Rotation2d.fromDegrees(theta));
        }

        this.drivetrain.drive(states);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        this.drivetrain.drive(0.0, 0.0, 0.0, false);
    }
}
