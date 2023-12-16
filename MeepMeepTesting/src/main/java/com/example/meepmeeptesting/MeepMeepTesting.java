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
                        drive.trajectorySequenceBuilder(new Pose2d(-37.74, -62.82, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-36.55, -35), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-36.55, -53.07), Math.toRadians(90))
                                .setReversed(false)
                                .splineTo(new Vector2d(-48.03, -49.85), Math.toRadians(90))
                                .splineTo(new Vector2d(-40.98, -10.77), Math.toRadians(0.00))
                                .splineTo(new Vector2d(31.12, -10.37), Math.toRadians(0.00))
                                //Need to decrease velocity
                                .splineToConstantHeading(new Vector2d(44, -38), Math.toRadians(0.00))
                                .build()
                );
        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}