/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name = "AutoEncoderDriveByTwoMotor", group = "Pushbot")
public class AutoEnocoderDriveByTwoMotor extends LinearOpMode {

    private static final double EncoderNumberChangePerInch = 34;
    private static final double DRIVE_SPEED = 0.3;
    private static final double TURN_SPEED = 0.4;

    // Config for the robot
    private config robot = new config(this.telemetry);

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        robot.ConfigureRobtHardware(this.hardwareMap);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set up imu
        telemetry.addData("Status", "Setting up imu");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.left_front.getCurrentPosition(),
                robot.right_front.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //ACTION
        encoderDrive(DRIVE_SPEED, 30, 30, 5.0);
        sleep(3000);
        encoderDrive(DRIVE_SPEED, -30, -30, 5.0);

        //sleep(2000);
        //encoderDrive(DRIVE_SPEED, 50, 50, 5.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        /*
        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0);
        */
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private void turnByTime(double speed, double turnDegree, double timeoutS) {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double direction = angles.firstAngle;
        double targetDirection = angles.firstAngle + turnDegree;
        robot.left_front.setPower(TURN_SPEED);
        robot.left_back.setPower(TURN_SPEED);
        robot.right_front.setPower(TURN_SPEED);
        robot.right_back.setPower(TURN_SPEED);

        while (opModeIsActive() && (runtime.seconds() < timeoutS) && !robot.isAtTarget(10) &&
                Math.abs(angles.firstAngle - targetDirection) > 10) {

        }
        robot.left_front.setPower(0);
        robot.right_front.setPower(0);
        //robot.left_back.setPower(0);
        //robot.right_back.setPower(0);
    }

    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        // Determine new target position, and pass to motor controller
        //double leftTarget2 = leftInches * EncoderNumberChangePerInch*WrongEncoderNumber*WrongEncoderCorrection;

        // TODO: Casting like this may return 0
        int newLeftTarget = (int) (robot.left_front.getCurrentPosition() + Math.round(leftInches * EncoderNumberChangePerInch)),
                newRightTarget = (int) (robot.right_front.getCurrentPosition() + Math.round(rightInches * EncoderNumberChangePerInch));

        //newLeftTarget2 = robot.left_back.getCurrentPosition() + (int) (leftTarget2),
        //newRightTarget2 = robot.right_back.getCurrentPosition() + (int) (rightInches * EncoderNumberChangePerInch);

        robot.left_front.setTargetPosition(newLeftTarget);
        robot.right_front.setTargetPosition(newRightTarget);
        //robot.left_back.setTargetPosition(newLeftTarget2);
        //robot.right_back.setTargetPosition(newRightTarget2);

        robot.left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left_front.setPower(Math.abs(speed));
        robot.right_front.setPower(Math.abs(speed));
        //robot.left_back.setPower(Math.abs(speed));
        //robot.right_back.setPower(Math.abs(speed));

        // Reset the runtime
        runtime.reset();

        // Ensure that the opmode is still active, we're within the timeout, and all motors are within their discrepancy of their target.
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && !robot.isAtTarget(10)) {

            // Just let the motors run. Since their mode is set to RUN_TO_POSITION, they will advance to/retreat from their location to the target,
            // and change the power accordingly.

            // Due to discrepancies, we cannot use isBusy() because if the target position is off by at least 1 tick,
            // the motor will keep trying to advance to the target, and will overshoot it.

            // Just update telemetry with current positions, targets, and powers
            robot.updateTelemetry();
            /*
            telemetry.addData("LF current:", robot.left_front.getCurrentPosition())
                    .addData("LF target:%7d", newLeftTarget);
            //telemetry.addData("LB current :%7d",robot.left_back.getCurrentPosition())
            //        .addData("   target:%7d",newLeftTarget2);
            telemetry.addData("RF current:%7d", robot.right_front.getCurrentPosition())
                    .addData("   target:%7d", newRightTarget);
            //telemetry.addData("RB current:%7d",robot.right_back.getCurrentPosition())
            //       .addData("   target:%7d",newRightTarget2);

            telemetry.update();
            */

        }

        // This gets executed once the time limit has expired. or the motors have reached their targets

        // Stop all motion;
        robot.left_front.setPower(0);
        robot.right_front.setPower(0);
        //robot.left_back.setPower(0);
        //robot.right_back.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move

    }
}
