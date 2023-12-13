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
                                .splineTo(new Vector2d(-43.11, -39.94), Math.toRadians(-135.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-33.73, -56.55), Math.toRadians(-135))
                                .setReversed(false)
                                .turn(Math.toRadians(45))
                                .splineTo(new Vector2d(-34.30, -25.06), Math.toRadians(-90.00))
                                .splineTo(new Vector2d(-20.58, -11), Math.toRadians(0))
                                .splineTo(new Vector2d(-4.69, -11), Math.toRadians(0))
                                .splineTo(new Vector2d(16.25, -11), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(30.69, -11), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(46.72, -29.68), Math.toRadians(0.00))
                                .build()
                );
        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}