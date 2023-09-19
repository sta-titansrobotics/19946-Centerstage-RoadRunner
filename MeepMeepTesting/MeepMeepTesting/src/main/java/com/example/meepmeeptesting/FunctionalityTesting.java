package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FunctionalityTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.01, 30, 2.1322221755981445, Math.toRadians(180), 14.65)
                .setDimensions(16, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                                .forward(20)
                                .lineToSplineHeading(new Pose2d(-40, 20, Math.toRadians(180)))

                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3, () -> {})
                                .UNSTABLE_addTemporalMarkerOffset(6, () -> {})
                                .waitSeconds(6)

                                .lineToSplineHeading(new Pose2d(40, 20, Math.toRadians(0)))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}