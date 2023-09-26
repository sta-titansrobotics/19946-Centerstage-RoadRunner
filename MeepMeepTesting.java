package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(49, 49, (49.0/16.5)*0.80, Math.toRadians(180), 14.65)
                .setDimensions(17.75, 17.875)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-70, 10, Math.toRadians(0)))

                                .splineToConstantHeading(new Vector2d(-37, 22), Math.toRadians(0)) //change to a lower value
                                .addDisplacementMarker(() -> {
                                    //drop the pixel
                                })
                                .lineToLinearHeading(new Pose2d(-41.25, 22, Math.toRadians(90))) //back up and face the left AprilTag

                                //Using headings this way will ensure that the camera sees the left aprilTag first
                                .lineToConstantHeading(new Vector2d(-41.25, 42.5))
                                .lineToConstantHeading(new Vector2d(-41.25, 50.5),
                                        SampleMecanumDrive.getVelocityConstraint(8, 49, 14.65),
                                        SampleMecanumDrive.getAccelerationConstraint(49)
                                        )
                                .addDisplacementMarker(() -> {
                                    //drop into the backdrop
                                })
                                .lineToConstantHeading(new Vector2d(-41.25, 48))
                                .lineToLinearHeading(new Pose2d(-10, 48, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-10, 70, Math.toRadians(270)))
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