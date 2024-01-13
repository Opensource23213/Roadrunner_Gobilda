package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14.75)
                .setDimensions(16.75, 17)
                .setStartPose(new Pose2d(-35, 60, Math.toRadians(-90)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(-90)))
                                .forward(29)
                                .back(16)
                                .turn(Math.toRadians(-90))
                                .forward(22)
                                .strafeLeft(12)
                                .forward(4)
                                .back(4)
                                .strafeLeft(22)
                                .back (100)
                                .turn(Math.toRadians(180))
                                .strafeLeft(29)
                                .strafeRight(29)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}