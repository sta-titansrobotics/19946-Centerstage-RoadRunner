package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        int parkNum = 1 + (int)(Math.random() * ((3-1) + 1));
        System.out.println(parkNum);
        Pose2d vectorPark = new Pose2d(0, 0);

        switch (parkNum) {
            case 1:
                vectorPark = new Pose2d(-57.5, 12, Math.toRadians(180));
                break;
            case 2:
                vectorPark = new Pose2d(-35, 12, Math.toRadians(180));
                break;
            case 3:
                vectorPark = new Pose2d(-12, 12, Math.toRadians(180));
                break;

        }

        Pose2d finalVectorPark = vectorPark;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(26.01, 30, 2.1322221755981445, Math.toRadians(180), 14.65)
                .setDimensions(16, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 62.3, Math.toRadians(0)))

                                // preload cone
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> verticalServo.setPosition(1))
                                .UNSTABLE_addTemporalMarkerOffset(3, () -> servoScissor.setPosition(0.67))
                    `           .UNSTABLE_addTemporalMarkerOffset(6, () -> verticalServo.setPosition(0))
                                 */
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {})
                                .UNSTABLE_addTemporalMarkerOffset(3, () -> {})
                                .UNSTABLE_addTemporalMarkerOffset(6, () -> {})
                                .waitSeconds(6)

                                .lineToSplineHeading(new Pose2d(-11.5, 62.3, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-11.5, 11.5, Math.toRadians(0)))
                                .turn(Math.toRadians(-135))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {}) //moving lift, power = 0.8, encoderticks = 3300
                                .waitSeconds(1)
                                .forward(7.25)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {}) //moving vertical servo, position is set to 0 (servo has been lifted up_
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {}) //moving servo scissor to 0.5 (servo scissor/claw has opened to drop the cone into the junction stick)
                                .waitSeconds(1)
                                .back(7.25)
                                .turn(Math.toRadians(-45))

                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {}) //move lift back to original position
                                .waitSeconds(1)
                                // park
                                .lineToSplineHeading((finalVectorPark))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}