package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    double getDriveDistance();

    double getSteerAngle();

    double getVelocity();
    
    void set(double driveVoltage, double steerAngle);

    SwerveModulePosition getState();
}
