package org.firstinspires.ftc.teamcode.auto;


import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Autonomous
public class AprilTagAdvancedFunctionality extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory startLeftTraj = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-52, -16), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    //drop the pixel
                })
                .lineToLinearHeading(new Pose2d(-52, -20, Math.toRadians(0))) //back up and face the left AprilTag
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            //ADD OBJECT DETECTION CODE HERE. DETERMINE IF NEED TO GO TO THE LEFT, MIDDLE, RIGHT
            //ASSIGNING A STRING VALUE WITH THE TRAJ WILL LET THE CAMERA KNOW WHICH APRILTAG IT HAS TO SENSE
            //E.G. IF LEFT, THEN USE CONDITIONAL STATEMENTS TO ENSURE THAT THE CAMERA FOLLOWS THE APRILTAG WITH ID OF 1

            // New trajectory
            // int LEFT TRAJECTORY = 1
            // int MIDDLE TRAJECTORY = 2
            // int RIGHT TRAJECTORY = 3

            AprilTagDetection tag = null;
            if (tagProcessor.getDetections().size() > 0) {
                tag = tagProcessor.getDetections().get(0);
            }

            boolean leftTrajectory = true;
            boolean middleTrajectory = false;
            if (leftTrajectory) {
                drive.followTrajectoryAsync(startLeftTraj);

                //adjustedAngle = .getBearing()
                //Alternatively use trig to calculate the angle bearing using the x, y values:
                double x = tag.ftcPose.x;
                double y = tag.ftcPose.y;
                double adjustedAngle = Math.round(Math.atan(x/y));
                Trajectory leftTraj_toAprilTag = drive.trajectoryBuilder(startLeftTraj.end())
                        //Using headings this way will ensure that the camera sees the left aprilTag first
                        .lineToSplineHeading(new Pose2d(-60, -20, Math.toRadians(270)))
                        .lineToSplineHeading(new Pose2d(-70, -20, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(leftTraj_toAprilTag);
                //Line up the robot to the adjusted angle
                drive.turn(adjustedAngle);
                //move forward to the backdrop, slower velocity
                Trajectory leftTraj_toOutput = drive.trajectoryBuilder(leftTraj_toAprilTag.end())
                        .build();

            } else if (middleTrajectory) {

            } else { //right trajectory

            }



        }



    }
}
