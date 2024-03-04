// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class AngleCalculator {
    SwerveSubsystem swerveSubsystem;
    public AngleCalculator(SwerveSubsystem subsystem){
        swerveSubsystem = subsystem;
    }
    
    double m = 0.06677;
    double n = 0.33162;

/* 
    double m1 = -0.8;
    double n1 = 20.3;//20.6;
    double a1 = 0.8;//0.8;
    double k1 = 0.7;
*/
//y = 0.0736x2 - 0.9586x + 22.457

    /*double a = -0.7;
    double k = 1.3;
    double p = 19.8;
    double n1 = -4.1;*/


    double a = 0.0736;
    double b = -0.9586;
    double c = 22.457;

    public double calculateDistance(Translation2d target){
        return Math.sqrt(Math.pow(swerveSubsystem.getEstimatedPose().getX() - target.getX(), 2) + Math.pow(swerveSubsystem.getEstimatedPose().getY() - target.getY(), 2));
    }

    public double getRobotX(){
        return swerveSubsystem.getEstimatedPose().getX();
    }

    /*public double calculateShooterSpeed(Translation2d target){
        double distance = calculateDistance(target);
        return MathUtil.clamp((m * distance + n), -0.8, 0.8);
    }*/

    public double calculateIntakeAngle(Translation2d target){
        double x = calculateDistance(target);
        //return MathUtil.clamp((m1 * Math.atan(k1 * calculateDistance(target) - a1) + n1), 18.0, 21.6); //min max değerleri intake açısının değişmemeye başladığı değerler (21.2 - 19.0 gibi)
        //return MathUtil.clamp((a * Math.atan(k * calculateDistance(target) + n1) + p), 18.0, 21.6);
        return MathUtil.clamp(a*x*x + b*x + c, 18.0, 21.6);
    }
}
