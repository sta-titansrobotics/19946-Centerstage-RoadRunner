package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

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
                .setConstraints(56, 56, Math.toRadians(180), Math.toRadians(180), 14.65)
                .setDimensions(17.75, 17.875)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))

                                // preload
                                .splineTo(new Vector2d(-35, -20), Math.toRadians(90))
                                .splineTo(new Vector2d(-29, -10.2), Math.toRadians(60))
                                .waitSeconds(2)

                                // cycle 1
                                .setReversed(true)
                                .splineTo(new Vector2d(-34, -11.6), Math.toRadians(180))
                                .splineTo(new Vector2d(-60, -11.6), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(2)

                                .splineTo(new Vector2d(-40, -11.6), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -11.6), Math.toRadians(58))
                                .waitSeconds(2)

                                // cycle 2
                                .setReversed(true)
                                .splineTo(new Vector2d(-34, -11.6), Math.toRadians(180))
                                .splineTo(new Vector2d(-60, -11.6), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(2)

                                .splineTo(new Vector2d(-40, -11.6), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -11.6), Math.toRadians(58))
                                .waitSeconds(2)


                                // parking
                                .setReversed(true)
                                .splineTo(new Vector2d(-39, -11.6), Math.toRadians(180))
                                .setReversed(false)

                                .lineToSplineHeading(new Pose2d(-58.5, -12, Math.toRadians(0)))




                                .build()
                );

        Image img = null;
        try {
            img = ImageIO.read(new File("CustomCenterstageDiagramWhite.png"));
        }
        catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep.setBackground(img)
//  <following code you were using previously>
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}