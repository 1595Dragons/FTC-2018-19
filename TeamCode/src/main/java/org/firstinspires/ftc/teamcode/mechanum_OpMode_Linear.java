package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

/**
 * Created by Stephen Ogden on 9/20/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@TeleOp(name="Mechanum OpMode Linear", group="Test")
public class mechanum_OpMode_Linear extends LinearOpMode {

    // Declare the config file, that way we can use the pre-made fictions for cleaner code
    private config robot = new config(this.telemetry);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware(this.hardwareMap);

        //MOTORS
        double IOLeftServoClose = 0.1,IOLeftServoHalfOpen = 0.5, IOLeftServoOpen = 0.8;
        double IORightServoClose = 0.8, IORightServoHalfOpen = 0.5, IORightServoOpen = 0.1;
        double speedForTurn = 0.5, speedForMove =0.6, speedForSide = 0.9;

        //ColorSensor
        final double SCALE_FACTOR = 255;
        float hsvValuesLeft[] = {0F, 0F, 0F};
        float hsvValuesRight[] = {0F, 0F, 0F};

        // Wait for the start button to be pressed
        waitForStart();

        // Code that we want to run repeatedly
        while (opModeIsActive()) {

            // Copy paste from Henry's Iterative program, with some formatting changes :)

            // Setup a variable for each drive wheel to save power level for telemetry
            double left1Power, right1Power, left2Power, right2Power, allPower = 1;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double driveForward = gamepad1.left_stick_y*speedForMove, driveRightSide = gamepad1.left_stick_x*speedForSide;
            double turnRight = -gamepad1.right_stick_x*speedForTurn;


            left1Power = Range.clip((-driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
            right1Power = Range.clip((driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;
            left2Power = Range.clip((driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
            right2Power = Range.clip((-driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.left_front.setPower(left1Power);
            robot.right_front.setPower(right1Power);
            robot.left_back.setPower(left2Power);
            robot.right_back.setPower(right2Power);

            //
            if (gamepad1.b)
            {
                Color.RGBToHSV((int) (robot.sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (robot.sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (robot.sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValuesLeft);//Hue value is hsvValuesLeft[0]
                Color.RGBToHSV((int) (robot.sensorColorRight.red() * SCALE_FACTOR),
                        (int) (robot.sensorColorRight.green() * SCALE_FACTOR),
                        (int) (robot.sensorColorRight.blue() * SCALE_FACTOR),
                        hsvValuesRight);//Hue value is hsvValuesRight[0]
                if (hsvValuesLeft[0]>=25&&hsvValuesLeft[0]<=50) {
                    robot.IO_Servo_Left.setPosition(IOLeftServoHalfOpen);
                }
                else {
                    robot.IO_Servo_Left.setPosition(IOLeftServoOpen);
                }
                if (hsvValuesRight[0]>=25&&hsvValuesRight[0]<=35) {
                    robot.IO_Servo_Right.setPosition(IORightServoHalfOpen);
                }
                else {
                    robot.IO_Servo_Right.setPosition(IORightServoOpen);
                }
            }
            else{
                robot.IO_Servo_Left.setPosition(IOLeftServoClose);
                robot.IO_Servo_Right.setPosition(IORightServoClose);
            }

            // Update telemetry
            //robot.updateTelemetry();
            if (robot.left_front != null) {
                telemetry.addData("Left front power", String.format(Locale.US, "%.2f", robot.left_front.getPower()));
            }
            if (robot.right_front != null) {
                telemetry.addData("Right front power", String.format(Locale.US, "%.2f", robot.right_front.getPower()));
            }
            if (robot.left_back != null) {
                telemetry.addData("Left back power", String.format(Locale.US, "%.2f", robot.left_back.getPower()));
            }
            if (robot.right_back != null) {
                telemetry.addData("Right back power", String.format(Locale.US, "%.2f", robot.right_back.getPower()));
            }
            telemetry.update();

        }

    }
}
