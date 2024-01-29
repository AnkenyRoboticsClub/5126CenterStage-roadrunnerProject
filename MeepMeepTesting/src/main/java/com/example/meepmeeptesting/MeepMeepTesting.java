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
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37.77, 62.9, Math.toRadians(-90.00)))
                                .splineTo(new Vector2d(-38.5, 39.94), Math.toRadians(-119.00))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 59.29), Math.toRadians(-267.51))
                                .setReversed(false)
                                .splineTo(new Vector2d(-20, 12), Math.toRadians(0))
                                .splineTo(new Vector2d(29.97, 12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(41, 29.68), Math.toRadians(0.00))
                                .build()
                );
        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}