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
                        drive.trajectorySequenceBuilder(new Pose2d(14.95, -63.62, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(9.5, -39), Math.toRadians(135.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(23, -53), Math.toRadians(90))
                                .setReversed(false)
                                .splineTo(new Vector2d(44, -29.68), Math.toRadians(0.00))
                                .turn(Math.toRadians(-90))
                                .setReversed(true)
                                .lineTo(new Vector2d(44,-60))
                                .build()
                );
        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}