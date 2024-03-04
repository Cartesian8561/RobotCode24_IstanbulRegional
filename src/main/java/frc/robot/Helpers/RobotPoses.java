// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** 
 * Nota Pozisyonlari red alliance driver station'una gore yazilmistir.
 * Driver stationa gore sol taraf isteniyorsa sol yazan pose'u al
 * 
 * uzaklar icin 1 en sag, 2 ortaya yakin olan sag
 * 
 * 
 * POSE 2D'DEN DEGER ALINIRKEN ANGLE RADIAN OLARAK VERILIR
*/
public class RobotPoses {
    public Pose2d autoGarantiShootPose(){
        return Constants.allianceFlip(new Pose2d(14.5, 5.4, new Rotation2d(Math.toRadians(0))));
    }

    public Pose2d autoAmpPose(){
        return Constants.allianceFlip(new Pose2d(0,0, new Rotation2d(Math.toRadians(0))));
    }

    public Pose2d yakinSagNotePose(){
        return Constants.allianceFlip(new Pose2d(14.2, 4.3, new Rotation2d(Math.toRadians(27.44))));
    }

    public Pose2d yakinOrtaNotePose(){//14
        return Constants.allianceFlip(new Pose2d(14.0, 5.4, new Rotation2d(Math.toRadians(0))));
    }

    public Pose2d yakinSolNotePose(){
        return Constants.allianceFlip(new Pose2d(13.8, 6.88, new Rotation2d(Math.toRadians(-35.44))));
    }

    public Pose2d uzakSag1notePose(){
        return Constants.allianceFlip(new Pose2d(8.85, 7.1, new Rotation2d(Math.toRadians(0))));
    }

    public Pose2d uzakSag2notePose(){
        return Constants.allianceFlip(new Pose2d(8.3, 5.3, new Rotation2d(Math.toRadians(0))));
    }

    public Pose2d uzakOrtaNotePose(){
        return Constants.allianceFlip(new Pose2d(8.6, 3.7, new Rotation2d(Math.toRadians(0))));
    }

    public Pose2d redStagePose(){
        return Constants.allianceFlip(new Pose2d(13.64, 4.105, new Rotation2d(Math.toRadians(27.44))));
    }

    public Pose2d redAmpPose(){
        return Constants.allianceFlip(new Pose2d(13.64, 4.105 + 2.9, new Rotation2d(Math.toRadians(-35.44))));
    }

    public Pose2d redCentrePose(){
        return Constants.allianceFlip(new Pose2d(13.64, 4.105 + 1.45, new Rotation2d(Math.toRadians(0))));
    }

    public Pose2d midPose1(){
        return Constants.allianceFlip(new Pose2d(8.27, 4.105 + 3.36, new Rotation2d(Math.toRadians(15.44)))); // rotationı götümden salladım
    }

    public Pose2d midPose2(){
        return Constants.allianceFlip(new Pose2d(8.27, 4.105 + 1.68, new Rotation2d(0)));
    }

    public Pose2d midPose3(){
        return Constants.allianceFlip(new Pose2d(8.27, 4.105, new Rotation2d(0)));
    }

    public Pose2d midPose4(){
        return Constants.allianceFlip(new Pose2d(8.27, 4.105 - 1.68, new Rotation2d(Math.toRadians(-15.44))));
    }

    public Pose2d midPose5(){
        return Constants.allianceFlip(new Pose2d(8.27, 4.105 - 3.36, new Rotation2d(Math.toRadians(-27.44))));
    }
}
