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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "Auto Imu with encoder", group = "Pushbot")
public class Auto_ImuWithEncoder extends LinearOpMode {

    private static final double EncoderNumberChangePerInch = 34;

    private static final double DRIVE_SPEED = .15, TURN_SPEED = .15, ARM_SPEED = .8, SIDE_SPEED = .25;
    private static final double MoveForOneDegree = 0.25;

    // Config for the robot
    private Config robot = new Config(this);

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        // Setup robot hardware
        robot.ConfigureRobtHardware();


        // Send telemetry message to signify robot waiting;
        robot.status("Resetting motors");
        robot.resetMotors(robot.left_back, robot.left_front, robot.right_back, robot.right_front, robot.armMotorL, robot.armMotorR);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //set up imu
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.initialize(parameters);
        sleep(5000);
        TurnByEncoder(TURN_SPEED,0,angles,4);

        //robot.InitializeVision(this.hardwareMap);
        //robot.StartTrackingVisionTargets();

        //robot.setupGoldDetector(this.hardwareMap);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        // Determine new target position, and pass to motor controller

        int newLeftTarget = robot.left_front.getCurrentPosition() + (int) (leftInches * EncoderNumberChangePerInch),
                newRightTarget = robot.right_front.getCurrentPosition() + (int) (rightInches * EncoderNumberChangePerInch),
                newLeftTarget2 = robot.left_back.getCurrentPosition() + (int) (leftInches * EncoderNumberChangePerInch),
                newRightTarget2 = robot.right_back.getCurrentPosition() + (int) (rightInches * EncoderNumberChangePerInch);

        robot.left_front.setTargetPosition(newLeftTarget);
        robot.right_front.setTargetPosition(newRightTarget);
        robot.left_back.setTargetPosition(newLeftTarget2);
        robot.right_back.setTargetPosition(newRightTarget2);

        robot.left_front.setPower(Math.abs(speed));
        robot.right_front.setPower(Math.abs(speed));
        robot.left_back.setPower(Math.abs(speed));
        robot.right_back.setPower(Math.abs(speed));

        // Reset the runtime
        runtime.reset();

        // Ensure that the opmode is still active, we're within the timeout, and all motors are within their discrepancy of their target.
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && !robot.isThere(10, robot.left_front, robot.right_front, robot.left_back, robot.right_back)) {

            // Just let the motors run. Since their mode is set to RUN_TO_POSITION, they will advance to/retreat from their location to the target,
            // and change the power accordingly.

            // Due to discrepancies, we cannot use isBusy() because if the target position is off by at least 1 tick,
            // the motor will keep trying to advance to the target, and will overshoot it.

            // Just update telemetry with current positions, targets, and powers
            robot.updateTelemetry();

        }

        // This gets executed once the time limit has expired. or the motors have reached their targets

        // Stop all motion;
        robot.left_front.setPower(0);
        robot.right_front.setPower(0);
        robot.left_back.setPower(0);
        robot.right_back.setPower(0);

    }

    private void distinctDrive(double speed, double LFInches, double LBInches, double RFInches, double RBInches, double timeoutS) {

        // Determine new target position, and pass to motor controller

        int newLeftTarget = robot.left_front.getCurrentPosition() + (int) (LFInches * EncoderNumberChangePerInch),
                newRightTarget = robot.right_front.getCurrentPosition() + (int) (RFInches * EncoderNumberChangePerInch),
                newLeftTarget2 = robot.left_back.getCurrentPosition() + (int) (LBInches * EncoderNumberChangePerInch),
                newRightTarget2 = robot.right_back.getCurrentPosition() + (int) (RBInches * EncoderNumberChangePerInch);

        robot.left_front.setTargetPosition(newLeftTarget);
        robot.right_front.setTargetPosition(newRightTarget);
        robot.left_back.setTargetPosition(newLeftTarget2);
        robot.right_back.setTargetPosition(newRightTarget2);

        robot.left_front.setPower(Math.abs(speed));
        robot.right_front.setPower(Math.abs(speed));
        robot.left_back.setPower(Math.abs(speed));
        robot.right_back.setPower(Math.abs(speed));

        // Reset the runtime
        runtime.reset();

        // Ensure that the opmode is still active, we're within the timeout, and all motors are within their discrepancy of their target.
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && !robot.isThere(10, robot.left_front, robot.right_front, robot.left_back, robot.right_back)) {

            // Just let the motors run. Since their mode is set to RUN_TO_POSITION, they will advance to/retreat from their location to the target,
            // and change the power accordingly.

            // Due to discrepancies, we cannot use isBusy() because if the target position is off by at least 1 tick,
            // the motor will keep trying to advance to the target, and will overshoot it.

            // Just update telemetry with current positions, targets, and powers
            robot.updateTelemetry();
        }

        // This gets executed once the time limit has expired. or the motors have reached their targets

        // Stop all motion;
        robot.left_front.setPower(0);
        robot.right_front.setPower(0);
        robot.left_back.setPower(0);
        robot.right_back.setPower(0);

    }

    private void armDrive(double speed, int armUp, double timeoutS) {

        robot.armMotorL.setTargetPosition(armUp);
        robot.armMotorR.setTargetPosition(armUp);

        robot.armMotorR.setPower(speed);
        robot.armMotorL.setPower(speed);

        // Reset the runtime
        runtime.reset();

        // Ensure that the opmode is still active, we're within the timeout, and all motors are within their discrepancy of their target.
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && !robot.isThere(5, robot.armMotorR, robot.armMotorL)) {

            // Just let the motors run. Since their mode is set to RUN_TO_POSITION, they will advance to/retreat from their location to the target,
            // and change the power accordingly.

            // Due to discrepancies, we cannot use isBusy() because if the target position is off by at least 1 tick,
            // the motor will keep trying to advance to the target, and will overshoot it.

            // Just update telemetry with current positions, targets, and powers
            robot.updateTelemetry();

        }

        // This gets executed once the time limit has expired, or when the motors have reached their targets

        // Stop all motion;
        robot.armMotorL.setPower(0);
        robot.armMotorR.setPower(0);

    }
    private void TurnByImu(double speed, float turnToAngle,BNO055IMU imu2, double timeoutS) {

        Orientation angles2;
        angles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double kp=0.007, ki=0.005, kd=0.0001;
        double lastTime=0,accumulation=0,lastAngle=angles2.firstAngle;
        double lastDifference=0;
        double output=0;

        // Reset the runtime
        runtime.reset();
        while(runtime.seconds()<timeoutS && ((angles2.firstAngle-turnToAngle)>=1||(angles2.firstAngle-turnToAngle)<=-1))
        {
            angles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if ((lastDifference>0&&(angles2.firstAngle-turnToAngle)<=0)
                    ||((lastDifference<0&&(angles2.firstAngle-turnToAngle)>=0)))
            {
                accumulation=0;
            }
            accumulation+=(runtime.seconds()-lastTime)*(angles2.firstAngle-turnToAngle);
            output=kp*(angles2.firstAngle-turnToAngle)+ki*accumulation+kd*(angles2.firstAngle-lastAngle)/(runtime.seconds()-lastTime);
            lastTime=runtime.seconds();
            lastAngle=angles2.firstAngle;
            lastDifference=angles2.firstAngle-turnToAngle;
            robot.left_front.setPower(output*speed);
            robot.left_back.setPower(output*speed);
            robot.right_front.setPower(-output*speed);
            robot.right_back.setPower(-output*speed);
        }
        // Stop all motion;
        robot.left_front.setPower(0);
        robot.left_back.setPower(0);
        robot.right_front.setPower(0);
        robot.right_back.setPower(0);
    }
    private void TurnByEncoder(double speed, float turnToAngle,Orientation angles2, double timeoutS)
    {
        double difference=angles2.firstAngle-turnToAngle;
        if(difference>180)
        {
            difference=difference-360;
        }
        telemetry.addData("difference",difference);
        telemetry.update();
        encoderDrive(speed,difference*MoveForOneDegree,-difference*MoveForOneDegree,timeoutS);
    }
}
