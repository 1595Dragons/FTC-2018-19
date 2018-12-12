package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Size;

import java.util.Locale;

/**
 * Created by Stephen Ogden on 9/13/18.
 * Modified on 12/1/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
class Config {

    // DcMotors and servos used on the robot
    private final int EncoderNumberChangePerInch = 34;
    DcMotor left_front, right_front, left_back, right_back, IO_Motor, armMotorL, armMotorR, armMotorExtend;
    Servo IO_Servo_Left, IO_Servo_Right;


    // Gold detector object
    GoldDetector goldDetector;


    // Gyro / IMU
    BNO055IMU imu;


    // A timer object
    private ElapsedTime timer = new ElapsedTime();


    // Get the important bits from the opMode
    private LinearOpMode OpMode;


    Config(LinearOpMode OpMode) {
        this.OpMode = OpMode;
    }


    /**
     * Goes through the configuration of the robot, and sets up all the motors and whatnot.
     * Even updating the telemetry :)
     */
    void ConfigureRobtHardware(boolean setupIMU) {

        // Declare and setup left_front
        this.status("Configuring left front motor");
        this.left_front = OpMode.hardwareMap.dcMotor.get("left front");
        this.left_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.left_front.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.left_front.setMode(RunMode.RUN_USING_ENCODER);
        this.left_front.setDirection(Direction.FORWARD);


        // Declare and setup right_front
        this.status("Configuring right front motor");
        this.right_front = OpMode.hardwareMap.dcMotor.get("right front");
        this.right_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.right_front.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.right_front.setMode(RunMode.RUN_USING_ENCODER);
        this.right_front.setDirection(Direction.REVERSE);


        // Declare and setup left_back
        this.status("Configuring left back motor");
        this.left_back = OpMode.hardwareMap.dcMotor.get("left back");
        this.left_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.left_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.left_back.setMode(RunMode.RUN_USING_ENCODER);
        this.left_back.setDirection(Direction.FORWARD);


        // Declare and setup right_back
        this.status("Configuring right back motor");
        this.right_back = OpMode.hardwareMap.dcMotor.get("right back");
        this.right_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.right_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.right_back.setMode(RunMode.RUN_USING_ENCODER);
        this.right_back.setDirection(Direction.REVERSE);


        // Declare and setup armMotorL
        this.status("Configuring left arm");
        this.armMotorL = OpMode.hardwareMap.dcMotor.get("arm motor left");
        this.armMotorL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.armMotorL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.armMotorL.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.armMotorL.setDirection(Direction.REVERSE);


        // Declare and setup armMotorR
        this.status("Configuring right arm");
        this.armMotorR = OpMode.hardwareMap.dcMotor.get("arm motor right");
        this.armMotorR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.armMotorR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.armMotorR.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.armMotorR.setDirection(Direction.FORWARD);


        // Declare and setup arm extender motor
        this.status("Configuring arm extender");
        this.armMotorExtend = OpMode.hardwareMap.dcMotor.get("arm motor extend");
        this.armMotorExtend.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.armMotorExtend.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.armMotorExtend.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.armMotorExtend.setDirection(Direction.FORWARD);


        // Declare and setup Intake Motor
        this.status("Configuring Intake Motor");
        this.IO_Motor = OpMode.hardwareMap.dcMotor.get("IO motor");
        this.IO_Motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.IO_Motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.IO_Motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.IO_Motor.setDirection(Direction.FORWARD);


        // Declare the left servo for the intake
        this.status("Setting up left servo");
        this.IO_Servo_Left = OpMode.hardwareMap.servo.get("IO Servo Left");


        // Declare the right servo for the intake
        this.status("Setting up right servo");
        this.IO_Servo_Right = OpMode.hardwareMap.servo.get("IO Servo Right");


        if (setupIMU) {
            this.status("Setting up imu...");
            final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            this.imu = OpMode.hardwareMap.get(BNO055IMU.class, "imu");
            this.imu.initialize(parameters);
        }


        // Update telemetry to signal done!
        this.status("Done!");
    }


    /**
     * Resets given DcMotors to 0, and sets their mode to an auto run to position. Also sets their targets to 0.
     *
     * @param motors Any given DcMotors that need to be reset
     */
    void resetMotorsForAutonomous(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(RunMode.RUN_TO_POSITION);
        }
    }


    /**
     * Returns if <i>any</i> of the provided motors are within its provided margin of error.
     *
     * @param error  The margin of error its allowed (in encoder ticks).
     * @param motors The motors to check.
     * @return Returns true if any of the motors are within its given margin of error. If all of them are outside the margin of error then it returns false.
     */
    private boolean isThere(int error, DcMotor... motors) {
        boolean reached = false;
        for (DcMotor motor : motors) {
            int delta = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
            if (delta <= error) {
                reached = true;
                break;
            }
        }
        return reached;
    }


    /**
     * Sets up the vision system (DogeCV) for detecting the gold (piss yellow) cube
     */
    void setupGoldDetector() {

        this.status("Creating gold detector");
        this.goldDetector = new GoldDetector();

        // Set the size of the camera
        this.status("Setting camera size");
        this.goldDetector.setAdjustedSize(new Size(270, 480));

        // Init the detector (try to use the defaults)
        this.status("Applying settings");
        this.goldDetector.init(OpMode.hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, false);
        this.goldDetector.useDefaults();

        // Apply the score based on color
        this.status("Applying color deviation scoring");
        this.goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.COLOR_DEVIATION;

        this.status("Done!");

    }


    /**
     * Searches for gold within the maximum amount of time, and returns whether or not it was found.
     *
     * @param milliseconds The amount of time in milliseconds that is allowed to be spent searching for the piece of gold.
     * @return Whether or not the gold cube was found in the given amount of time.
     */
    boolean searchForGold(int milliseconds) {
        this.timer.reset();
        while (this.timer.milliseconds() < milliseconds && this.OpMode.opModeIsActive()) {
            this.OpMode.telemetry.addData("Elapsed time", this.timer.milliseconds());
            this.OpMode.telemetry.addLine();
            if (this.goldDetector.isFound()) {
                return true;
            }
            this.OpMode.idle();
        }
        return false;
    }


    /**
     * Sets the telemetry status. This is usually used at the start, end, and init of programs.
     *
     * @param string The status of what is going on. Yep, that made sense.
     */
    void status(String string) {
        this.OpMode.telemetry.addData("Status", string);
        this.OpMode.telemetry.update();
    }


    /**
     * Gets the current angle of the robot from the built in IMU. We use the Y axis for this.
     *
     * @return The robot's current angle in degrees. This is rounded to the nearest whole number.
     */
    int getAngle() {
        return this.imu.isGyroCalibrated() ? Math.round(this.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).secondAngle) : 0;
    }


    /**
     * Runs the 2 arm motors to a provided position.
     *
     * @param speed    The speed the arm motors are set to run to.
     * @param armPos   The position that will be set as the motors target positions.
     * @param timeoutS The duration in seconds the function is allowed to run.
     */
    void armDrive(double speed, int armPos, int timeoutS) {

        // Check if the arm motors are RUN_TO_POSITION
        if (!this.armMotorL.getMode().equals(RunMode.RUN_TO_POSITION)) {
            this.armMotorL.setMode(RunMode.RUN_TO_POSITION);
        }
        if (!this.armMotorR.getMode().equals(RunMode.RUN_TO_POSITION)) {
            this.armMotorR.setMode(RunMode.RUN_TO_POSITION);
        }

        // Set the arm motors target positions and speed
        this.armMotorR.setTargetPosition(armPos);
        this.armMotorL.setTargetPosition(armPos);
        this.armMotorR.setPower(speed);
        this.armMotorL.setPower(speed);

        this.timer.reset();
        while (OpMode.opModeIsActive() && this.timer.seconds() < timeoutS) {

            // Check to see if the targeted position has been met. If it has, break out of the while loop
            if (this.isThere(5, this.armMotorL, this.armMotorR)) {
                break;
            }
        }


        this.armMotorR.setPower(0);
        this.armMotorL.setPower(0);

    }


    void autoTurnToDegree(double speed, int turnToAngle, int timeoutS) {

        double error, steer, P = 0.025d;

        this.right_back.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.right_front.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.left_back.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.left_front.setMode(RunMode.RUN_WITHOUT_ENCODER);

        this.timer.reset();
        while (this.OpMode.opModeIsActive() && this.timer.seconds() <= timeoutS) {
            error = this.getError(turnToAngle);

            steer = getSteer(error, P);

            if (Math.abs(error) <= 2) {
                this.resetMotorsForAutonomous(this.left_front, this.left_back, this.right_front, this.right_back);
                break;
            } else {
                this.left_front.setPower(-steer * speed);
                this.left_back.setPower(-steer * speed);
                this.right_front.setPower(steer * speed);
                this.right_back.setPower(steer * speed);

                this.OpMode.telemetry.addData("Turning to degree", turnToAngle);
                this.OpMode.telemetry.addLine();
                this.OpMode.telemetry.addData("Current angle", this.getAngle());
                this.OpMode.telemetry.update();
            }
        }
    }


    void autoDriveStraight(double speed, int inches, int currentAngle, int timeoutS) {

        int ticks = -inches * this.EncoderNumberChangePerInch;

        this.resetMotorsForAutonomous(this.left_back, this.right_back, this.right_front, this.left_front);

        double rightSpeed, leftSpeed, error, steer, P = 0.025d;

        this.left_front.setTargetPosition(ticks);
        this.right_front.setTargetPosition(ticks);
        this.left_back.setTargetPosition(ticks);
        this.right_back.setTargetPosition(ticks);


        this.timer.reset();
        while (this.OpMode.opModeIsActive() && this.timer.seconds() <= timeoutS) {

            if (this.isThere(this.EncoderNumberChangePerInch, this.left_front, this.left_back, this.right_front, this.right_back)) {
                this.resetMotorsForAutonomous(this.left_back, this.right_back, this.right_front, this.left_front);

                if (Math.abs(this.getError(currentAngle)) > 2) {
                    this.autoTurnToDegree(.5, currentAngle, (int) Math.round(timeoutS - this.timer.seconds()));
                }

                break;
            } else {

                error = this.getError(currentAngle);

                steer = this.getSteer(error, P);

                // If driving in reverse, the motor correction also needs to be reversed
                steer = steer < 0 ? steer * -1 : steer;

                leftSpeed = Range.clip(speed - steer, -speed, speed);
                rightSpeed = Range.clip(speed + steer, -speed, speed);

                this.left_front.setPower(leftSpeed);
                this.right_front.setPower(rightSpeed);
                this.left_back.setPower(leftSpeed);
                this.right_back.setPower(rightSpeed);

                this.OpMode.telemetry.addData("Angle", this.getAngle());
                this.OpMode.telemetry.addLine();
                this.OpMode.telemetry.addData("Left power", String.format(Locale.US, "%.4f", leftSpeed))
                        .addData("Right power", String.format(Locale.US, "%.4f", rightSpeed));
                this.OpMode.telemetry.addLine();
                this.OpMode.telemetry.addData("Target", ticks);
                this.OpMode.telemetry.addLine();
                this.OpMode.telemetry.addData("Left front position", this.left_front.getCurrentPosition())
                        .addData("Left back position", this.left_back.getCurrentPosition())
                        .addData("Right front position", this.right_front.getCurrentPosition())
                        .addData("Right back position", this.right_back.getCurrentPosition());
                this.OpMode.telemetry.update();

            }
        }
    }

    // FIXME
    void autoDriveSideways(double speed, int inches, int currentAngle, int timeoutS) {

        int ticks = inches * this.EncoderNumberChangePerInch;

        this.resetMotorsForAutonomous(this.left_front, this.right_back);

        double leftFrontPower, rightBackPower, rightFrontPower, leftBackPower, error, steer, P = .5;

        this.left_front.setTargetPosition(-ticks);
        this.right_back.setTargetPosition(-ticks);
        this.right_front.setTargetPosition(ticks);
        this.left_back.setTargetPosition(ticks);


        this.timer.reset();
        while (this.OpMode.opModeIsActive() && this.timer.seconds() < timeoutS) {

            if (this.isThere(2, this.left_front, this.left_back, this.right_front, this.right_back)) {
                this.resetMotorsForAutonomous(this.left_back, this.left_front, this.right_back, this.right_front);

                if (Math.abs(this.getError(currentAngle)) > 2) {
                    this.autoTurnToDegree(.6, currentAngle, (int) Math.round(timeoutS - this.timer.seconds()));
                }

                break;
            } else {

                error = this.getError(currentAngle);

                steer = this.getSteer(error, P);

                // If driving in reverse, the motor correction also needs to be reversed
                steer = steer < 0 ? -steer : steer;

                leftFrontPower = Range.clip(speed - steer, -speed, speed);
                rightBackPower = Range.clip(speed + steer, -speed, speed);

                rightFrontPower = -rightBackPower;
                leftBackPower = -leftFrontPower;

                this.left_front.setPower(leftFrontPower);
                this.right_back.setPower(rightBackPower);

                this.right_front.setPower(rightFrontPower);
                this.left_back.setPower(leftBackPower);


                this.OpMode.telemetry.addData("Angle", this.getAngle())
                        .addData("Steer", steer);
                this.OpMode.telemetry.addLine();
                this.OpMode.telemetry.addData("Left front power", String.format(Locale.US, "%.4f", leftFrontPower))
                        .addData("Right back power", String.format(Locale.US, "%.4f", rightBackPower))
                        .addData("Right front power", String.format(Locale.US, "%.4f", rightFrontPower))
                        .addData("Left back power", String.format(Locale.US, "%.4f", leftBackPower));
                this.OpMode.telemetry.addLine();
                this.OpMode.telemetry.addData("Target", ticks);
                this.OpMode.telemetry.addLine();
                this.OpMode.telemetry.addData("Left front position", this.left_front.getCurrentPosition())
                        .addData("Left back position", this.left_back.getCurrentPosition())
                        .addData("Right front position", this.right_front.getCurrentPosition())
                        .addData("Right back position", this.right_back.getCurrentPosition());
                this.OpMode.telemetry.update();
            }
        }
    }


    private double getError(int desiredAngle) {
        double error = this.imu.isGyroCalibrated() ? Math.round((desiredAngle) - (this.getAngle())) : 0;

        while (error > 180) {
            error -= 360;
        }
        while (error <= -180) {
            error += 360;
        }

        return error;
    }


    private double getSteer(double error, double PCoeff) {
        // Find the steer value. Subtract 180 to get the direction in case its actually negative
        return Range.clip((error) * PCoeff, -1, 1);
    }


    /**
     * Runs to a given position in inches via encoders.
     *
     * @param speed       The max speed allowed for the motors.
     * @param leftInches  The distance in inches that the left side needs to go.
     * @param rightInches The distance in inches that the right side needs to go.
     * @param timeoutS    The amount of time in seconds that the function is allowed to execute.
     */
    @Deprecated
    void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        // Its literally distinctDrive, but with 2 positions
        this.distinctDrive(speed, leftInches, leftInches, rightInches, rightInches, timeoutS);
    }


    /**
     * Sets the target position of the individual drive motors, and then approaches that point.
     *
     * @param speed    The maximum speed each motor is allowed to run at.
     * @param LFInches The left front motor's target position in inches.
     * @param LBInches The left back motor's target position in inches.
     * @param RFInches The right front motor's target position in inches.
     * @param RBInches The right back motor's target position in inches.
     * @param timeoutS The amount of time in seconds that the function is allowed to execute.
     */
    @Deprecated
    void distinctDrive(double speed, double LFInches, double LBInches, double RFInches, double RBInches, double timeoutS) {

        // Reset the motor encoders, and set them to RUN_TO_POSITION
        this.resetMotorsForAutonomous(this.left_front, this.left_back, this.right_front, this.right_back);

        // Set the individual drive motor positions
        this.left_front.setTargetPosition((int)Math.round(LFInches * EncoderNumberChangePerInch));
        this.right_front.setTargetPosition((int)Math.round(RFInches * EncoderNumberChangePerInch));
        this.left_back.setTargetPosition((int)Math.round(LBInches * EncoderNumberChangePerInch));
        this.right_back.setTargetPosition((int)Math.round(RBInches * EncoderNumberChangePerInch));

        // Set the motor speeds
        this.left_front.setPower(speed);
        this.right_front.setPower(speed);
        this.left_back.setPower(speed);
        this.right_back.setPower(speed);

        // Reset the runtime
        this.timer.reset();
        while (OpMode.opModeIsActive() && (this.timer.seconds() < timeoutS)) {

            // Check if the target has been reached
            if (this.isThere(4, this.left_back, this.left_front, this.right_back, this.right_front)) {
                // Break out of the while loop early
                break;
            }
        }

        // Stop all motion, and reset the motors
        this.resetMotorsForAutonomous(this.left_back, this.left_front, this.right_back, this.right_front);
    }
    void TurnByImu(double speed,int target, double timeOut)
    {
        double error =this.getError(target);
        double magicNumber = 0.27;
        encoderDrive(speed,-error*magicNumber, error*magicNumber, timeOut);
    }
}