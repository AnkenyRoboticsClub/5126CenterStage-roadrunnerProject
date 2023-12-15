package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String args[]){
        MeepMeep mm = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.74, -62.82, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-30.94, -36.05), Math.toRadians(45.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-45, -54.09), Math.toRadians(45))
                                .setReversed(false)
                                .splineTo(new Vector2d(-51.57, -25.84), Math.toRadians(100.01))
                                .splineTo(new Vector2d(-18.89, -10.31), Math.toRadians(0))
                                .splineTo(new Vector2d(27.89, -9.97), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(44, -44.85), Math.toRadians(0))
                                .build()
                );
        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}