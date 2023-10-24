package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class EncoderTestingPID extends LinearOpMode {

    //Set motor variables
    public DcMotor motorFL;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorBR;

    //Initializing encoder positions
    public int leftPos1;
    public int leftPos2;
    public int rightPos1;
    public int rightPos2;

    public static final int TICKS_PER_REVOLUTION = 1440; // Replace with your motor's ticks per revolution
    public static final double WHEEL_DIAMETER_INCHES = 4.0; // Replace with your wheel diameter
    public static final double  DRIVE_SPEED = 0.5; // Adjust the speed as needed
    public static final int TARGET_DISTANCE_INCHES = 12; // Replace with your target distance
    public static final double RADIUS = 1.875;

    double integralSum = 0;
    double Kp = 0.13;
    double Ki = 0;
    double Kd = 0.0001;
    double Kf = 0.2;

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize motors
        motorFL = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFR = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBR = hardwareMap.get(DcMotor.class, "motorBackRight");

        //set mode to stop and reset encoders -- resets encoders to the 0 position
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Reverse left side motors
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize the positions to zero, since the motor has not moved yet
        leftPos1 = 0;
        leftPos2 = 0;
        rightPos1 = 0;
        rightPos2 = 0;

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //any code after this command will not be executed until the match has started
        waitForStart();

        //can now set drive distance because of the function below; now we just need to input the distance
        //can also control the direction using the mecanum drivetrain directions here: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

        //Strafe diagonally left towards the
        PIDDrive(100, 100, 100, 100);
        //ex: this command will get the robot to strafe left for 9cm at a speed of 1
        PIDDrive(-100, 100, 100, -100);
        //
        PIDDrive(-280, -280, 280, 280);


        while (opModeIsActive()) {
            telemetry.addData("motorFL Encoder Position: ",motorFL.getCurrentPosition());
            telemetry.addData("motorBL Encoder Position: ",motorBL.getCurrentPosition());
            telemetry.addData("motorFR Encoder Position: ",motorFR.getCurrentPosition());
            telemetry.addData("motorBR Encoder Position: ",motorBR.getCurrentPosition());
            telemetry.update();
        }
    }

    public void PIDDrive(int leftTarget1, int leftTarget2, int rightTarget1, int rightTarget2) {
        //Just use as a reference conversion:
        double forwardTicks = 52.3;
        double strafeTicks = 54.05;

        int forwardTicks1 = (int) forwardTicks;
        int strafeTicks1 = (int) strafeTicks;

        while (opModeIsActive()) {
            motorFL.setPower(PIDControl(leftTarget1, motorFL.getCurrentPosition()));
            motorBL.setPower(PIDControl(leftTarget2, motorBL.getCurrentPosition()));
            motorFR.setPower(PIDControl(rightTarget1, motorFR.getCurrentPosition()));
            motorBR.setPower(PIDControl(rightTarget2, motorBR.getCurrentPosition()));
        }
    }

    /*
    public void turnRight(int leftTarget1, int leftTarget2, int rightTarget1, int rightTarget2) {
        double distance = (angle / 360.0) * (2.0 * Math.PI * RADIUS);
        int convertedDistance = (int) distance;

        //Just use as a reference conversion:
        double forwardTicks = 52.3;
        double strafeTicks = 54.05;

        int forwardTicks1 = (int) forwardTicks;
        int strafeTicks1 = (int) strafeTicks;

        while (opModeIsActive()) {
            motorFL.setPower(PIDControl(leftTarget1, motorFL.getCurrentPosition()));
            motorBL.setPower(PIDControl(leftTarget2, motorBL.getCurrentPosition()));
            motorFR.setPower(PIDControl(rightTarget1, motorFR.getCurrentPosition()));
            motorBR.setPower(PIDControl(rightTarget2, motorBR.getCurrentPosition()));
        }
    }

    public void turnLeft(int angle) {
        double distance = (angle / 360.0) * (2.0 * Math.PI * RADIUS);
        int convertedDistance = (int) distance;

        // Reset encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for all motors
        motorFrontLeft.setTargetPosition(-convertedDistance);
        motorFrontRight.setTargetPosition(convertedDistance);
        motorBackLeft.setTargetPosition(-convertedDistance);
        motorBackRight.setTargetPosition(convertedDistance);

        // Run to target position
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(PID(convertedDistance, motorFrontLeft.getCurrentPosition()));
        motorFrontRight.setPower(PID(convertedDistance, motorFrontRight.getCurrentPosition()));
        motorBackLeft.setPower(PID(convertedDistance, motorBackLeft.getCurrentPosition()));
        motorBackRight.setPower(PID(convertedDistance, motorBackLeft.getCurrentPosition()));

        // Wait until all motors reach the target position
        while (
                motorFrontLeft.isBusy() &&
                        motorFrontRight.isBusy() &&
                        motorBackLeft.isBusy() &&
                        motorBackRight.isBusy()) {
            // You can add other actions or checks here
            // For example, use sensors to detect obstacles or perform other tasks
        }

    }
*/
    public double PIDControl(double setPosition, double currentPosition) {
        double error = setPosition - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (setPosition * Kf);
    }
}
