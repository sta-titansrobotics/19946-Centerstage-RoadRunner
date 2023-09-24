package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);


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

                                .splineToConstantHeading(new Vector2d(-45, -16), Math.toRadians(90)) //change to a lower value
                                .addDisplacementMarker(() -> {
                                    //drop the pixel
                                })
                                .lineToLinearHeading(new Pose2d(-52, -20, Math.toRadians(0))) //back up and face the left AprilTag

                                //Using headings this way will ensure that the camera sees the left aprilTag first
                                .lineToSplineHeading(new Pose2d(-60, -20, Math.toRadians(270)))
                                .lineToSplineHeading(new Pose2d(-70, -20, Math.toRadians(180)))
                                .build()
                );

        Image img = null;
        try {
            img = ImageIO.read(new File("C:\\Users\\kaden\\StudioProjects\\19446-Centerstage-RoadRunner\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\field-2023-official.png"));
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