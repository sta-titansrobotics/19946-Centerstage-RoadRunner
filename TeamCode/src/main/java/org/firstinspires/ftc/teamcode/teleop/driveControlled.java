// FINALIZED DRIVE CONTROLLED, DO NOT TOUCH ANYTHING HERE //
/*
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class driveControlled extends LinearOpMode {
    @Override
    public void runOpMode() {

        //Moving
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //Odometers?
        DcMotor leftOdom = hardwareMap.get(DcMotor.class, "leftOdom");
        DcMotor rightOdom = hardwareMap.get(DcMotor.class, "rightOdom");
        DcMotor centerOdom = hardwareMap.get(DcMotor.class, "centerOdom");

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        double p = 0.0, i = 0.0, d = 0.0;
        double p_theta = 0.0, i_theta = 0.0, d_theta = 0.0;
        //Initialize starting positions
        double prevLeft = 0.0, prevRight = 0.0, prevCenter = 0.0;
        double currentX = 0.0, currentY = 0.0, heading = 0.0; // Initial angle is zero degrees, with (x, y) = (0, 0)

        double ticksPerRevolution = 1425.1;
        double robotWidth = 2.0; // Change this later.
        double wheelDiameter = 1.875 * 2; //In inches

        double trackWidth = 16.5; //in inches
        double forwardOffset = 2.0; //Center of the robot to horizontal odometer. Use 0 if no horizontal sensor

        PIDController xControl = new PIDController(p, i, d);
        PIDController yControl = new PIDController(p, i, d);
        PIDController thetaControl = new PIDController(p_theta, i_theta, d_theta);

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            //Driving

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!

            //STRAFING VARIABLE
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

            //THIS IS THE TURNING VARIABLE
            double rx = gamepad1.right_stick_x;

            /*
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            xControl.setPID(p, i, d);
            yControl.setPID(p, i, d);
            thetaControl.setPID(p_theta, i_theta, d_theta);

            /* WITHOUT ODOMETRY
            // Get encoder values for each mecanum wheel
            double frontLeftEncoderTicks = motorFL.getCurrentPosition(); // Read front left wheel encoder ticks
            double frontRightEncoderTicks = motorFR.getCurrentPosition(); // Read front right wheel encoder ticks
            double rearLeftEncoderTicks = motorBL.getCurrentPosition(); // Read rear left wheel encoder ticks
            double rearRightEncoderTicks = motorBR.getCurrentPosition(); // Read rear right wheel encoder ticks

            // Calculate distances traveled by each wheel
            double frontLeftDistance = (frontLeftEncoderTicks / ticksPerRevolution) * (Math.PI * wheelDiameter);
            double frontRightDistance = (frontRightEncoderTicks / ticksPerRevolution) * (Math.PI * wheelDiameter);
            double rearLeftDistance = (rearLeftEncoderTicks / ticksPerRevolution) * (Math.PI * wheelDiameter);
            double rearRightDistance = (rearRightEncoderTicks / ticksPerRevolution) * (Math.PI * wheelDiameter);

            // Calculate the combined distance for x and y movement
            double averageLeft = (frontLeftDistance + rearLeftDistance) / 2.0;
            double averageRight = (frontRightDistance + rearRightDistance) / 2.0;

            //Calculate change in values of x, y, t:

            // Calculate changes in x, y, and theta based on mecanum drive kinematics
            double deltaX = (averageLeft + averageRight) / 2.0 * Math.cos(currentTheta); //Change in x since x = cos(t)
            double deltaY = (averageLeft + averageRight) / 2.0 * Math.sin(currentTheta); //Change in y since y = cos(t)
            // Calculate the rotational movement (change in theta)
            double deltaTheta = (averageRight - averageLeft) / robotWidth; // robotWidth is the distance between the wheels

            //Calculate the speed:
            double speed = Range.clip(Math.max(Math.abs(x), Math.abs(y)), -1.0, 1.0);



            //Change in Encoder positions:
            double deltaLeftOdom = leftOdom.getCurrentPosition() - prevLeft;
            double deltaRightOdom = rightOdom.getCurrentPosition() - prevRight;
            double deltaCenterOdom = centerOdom.getCurrentPosition() - prevCenter;

            //Calculate change in angle:
            double deltaTheta = (deltaLeftOdom - deltaRightOdom) / trackWidth;

            //Calculate delta positions:
            double deltaMiddlePos = (deltaLeftOdom + deltaRightOdom) / 2.0;
            double deltaPerpPos = deltaCenterOdom - forwardOffset * deltaTheta;

            //Calculate deltaX, deltaY
            double deltaX = deltaMiddlePos * Math.cos(heading) - deltaPerpPos * Math.sin(heading);
            double deltaY = deltaMiddlePos * Math.sin(heading) - deltaPerpPos * Math.cos(heading);

            //Update x, y, theta positions:
            currentX += deltaX;
            currentY += deltaY;
            heading += deltaTheta;

            //Calculate PID values:
            double X = xControl.calculate(x, currentX);
            double Y = yControl.calculate(y, currentY);
            double Theta = thetaControl.calculate(rx, heading);

            //Set relative powers
            motorFL.setPower(X + Y + Theta);
            motorBL.setPower(X - Y + Theta);
            motorFR.setPower(X - Y - Theta);
            motorBR.setPower(X + Y - Theta);

            //Update previous positions
            prevLeft = leftOdom.getCurrentPosition();
            prevRight = rightOdom.getCurrentPosition();
            prevCenter = centerOdom.getCurrentPosition();

            if (gamepad1.dpad_up) {
                motorFL.setPower(1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(1);
            }

            if (gamepad1.dpad_down) {
                motorFL.setPower(-1);
                motorBL.setPower(-1);
                motorFR.setPower(-1);
                motorBR.setPower(-1);
            }

            if (gamepad1.dpad_left) {
                motorFL.setPower(-1);
                motorBL.setPower(1);
                motorFR.setPower(1);
                motorBR.setPower(-1);
            }

            if (gamepad1.dpad_right) {
                motorFL.setPower(1);
                motorBL.setPower(-1);
                motorFR.setPower(-1);
                motorBR.setPower(1);
            }

            // Add sensors to telemetry???
            telemetry.addData("LF Power:", motorFL.getPower());
            telemetry.addData("LB Power:", motorBL.getPower());
            telemetry.addData("RF Power:", motorFR.getPower());
            telemetry.addData("RB Power:", motorBR.getPower());
            telemetry.addData("LF Position:", motorFL.getCurrentPosition());
            telemetry.addData("LB Position:", motorBL.getCurrentPosition());
            telemetry.addData("RF Position:", motorFR.getCurrentPosition());
            telemetry.addData("RB Position:", motorBR.getCurrentPosition());
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Heading: ", heading);
            telemetry.addData("X Control Output", X);
            telemetry.addData("Y Control Output", Y);
            telemetry.addData("Theta Control Output", Theta);

            telemetry.update();
        }
    }
}
*/
