package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Size;

import java.util.ArrayList;
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


    // Version 2 color sensor
    ColorSensor sensorColorLeft, sensorColorRight;
    DistanceSensor sensorDistanceLeft, sensorDistanceRight;


    // Gold detector object
    GoldDetector goldDetector;


    // Gyro / IMU
    BNO055IMU imu;


    // A timer object
    private ElapsedTime timer = new ElapsedTime();


    // Create a list of all the Hardware devices, mainly for telemetry :)
    private ArrayList<HardwareDevice> Devices = new ArrayList<>();


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
        this.Devices.add(left_front);


        // Declare and setup right_front
        this.status("Configuring right front motor");
        this.right_front = OpMode.hardwareMap.dcMotor.get("right front");
        this.right_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.right_front.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.right_front.setMode(RunMode.RUN_USING_ENCODER);
        this.right_front.setDirection(Direction.REVERSE);
        this.Devices.add(right_front);


        // Declare and setup left_back
        this.status("Configuring left back motor");
        this.left_back = OpMode.hardwareMap.dcMotor.get("left back");
        this.left_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.left_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.left_back.setMode(RunMode.RUN_USING_ENCODER);
        this.left_back.setDirection(Direction.FORWARD);
        this.Devices.add(left_back);


        // Declare and setup right_back
        this.status("Configuring right back motor");
        this.right_back = OpMode.hardwareMap.dcMotor.get("right back");
        this.right_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.right_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.right_back.setMode(RunMode.RUN_USING_ENCODER);
        this.right_back.setDirection(Direction.REVERSE);
        this.Devices.add(right_back);


        // Declare and setup armMotorL
        this.status("Configuring left arm");
        this.armMotorL = OpMode.hardwareMap.dcMotor.get("arm motor left");
        this.armMotorL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.armMotorL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.armMotorL.setMode(RunMode.RUN_USING_ENCODER);
        this.armMotorL.setDirection(Direction.REVERSE);
        this.Devices.add(armMotorL);


        // Declare and setup armMotorR
        this.status("Configuring right arm");
        this.armMotorR = OpMode.hardwareMap.dcMotor.get("arm motor right");
        this.armMotorR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.armMotorR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.armMotorR.setMode(RunMode.RUN_USING_ENCODER);
        this.armMotorR.setDirection(Direction.FORWARD);
        this.Devices.add(armMotorR);


        // Declare and setup arm extender motor
        this.status("Configuring arm extender");
        this.armMotorExtend = OpMode.hardwareMap.dcMotor.get("arm motor extend");
        this.armMotorExtend.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.armMotorExtend.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.armMotorExtend.setMode(RunMode.RUN_USING_ENCODER);
        this.armMotorExtend.setDirection(Direction.FORWARD);
        this.Devices.add(armMotorExtend);


        // Declare and setup Intake Motor
        this.status("Configuring Intake Motor");
        this.IO_Motor = OpMode.hardwareMap.dcMotor.get("IO motor");
        this.IO_Motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.IO_Motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.IO_Motor.setMode(RunMode.RUN_USING_ENCODER);
        this.IO_Motor.setDirection(Direction.FORWARD);
        this.Devices.add(IO_Motor);


        // Declare the left servo for the intake
        this.status("Setting up left servo");
        this.IO_Servo_Left = OpMode.hardwareMap.servo.get("IO Servo Left");
        this.Devices.add(IO_Servo_Left);


        // Declare the right servo for the intake
        this.status("Setting up right servo");
        this.IO_Servo_Right = OpMode.hardwareMap.servo.get("IO Servo Right");
        this.Devices.add(IO_Servo_Right);


        // Decalre and setup the color sensors
        this.status("Setting up color sensor");
        this.sensorColorLeft = OpMode.hardwareMap.colorSensor.get("color sensor left");
        this.sensorDistanceLeft = OpMode.hardwareMap.get(DistanceSensor.class, "color sensor left");
        this.sensorColorRight = OpMode.hardwareMap.colorSensor.get("color sensor right");
        this.sensorDistanceRight = OpMode.hardwareMap.get(DistanceSensor.class, "color sensor right");
        this.Devices.add(sensorColorLeft);
        this.Devices.add(sensorDistanceLeft);
        this.Devices.add(sensorColorRight);
        this.Devices.add(sensorDistanceRight);


        if (setupIMU) {
            this.status("Setting up imu...");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.FAST;
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
    void resetMotors(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(RunMode.RUN_TO_POSITION);
        }
    }


    /**
     * Updates the telemetry automatically output of all important objects, if they are not null.
     * <p>
     * <p>
     * DcMotors display power, encoder values, and targets.
     * <p>
     * Servos display targets.
     * <p>
     * Color sensors display their current RGBA values.
     * <p>
     * Distance sensors display their distance in inches.
     * <p>
     * IMU (If the gyro is calibrated) displays the XYZ angles in degrees.
     * <p>
     * And the gold detector displays if and where it found gold.
     */
    void updateTelemetry() {

        for (HardwareDevice device : Devices) {
            if (device != null) {
                if (device instanceof DcMotor) {
                    DcMotor motor = (DcMotor) device;
                    this.OpMode.telemetry.addData(motor.getDeviceName() + " power", String.format(Locale.US, "%.2f", motor.getPower()))
                            .addData(motor.getDeviceName() + " position", motor.getCurrentPosition())
                            .addData(motor.getDeviceName() + " target (Displacement)", String.format(Locale.US, "%s (%s)",
                                    motor.getTargetPosition(), Math.abs(motor.getCurrentPosition() - motor.getTargetPosition())));
                    this.OpMode.telemetry.addLine();
                } else if (device instanceof Servo) {
                    Servo servo = (Servo) device;
                    this.OpMode.telemetry.addData(servo.getDeviceName() + " target position", servo.getPosition());
                    this.OpMode.telemetry.addLine();
                } else if (device instanceof ColorSensor) {
                    ColorSensor colorSensor = (ColorSensor) device;
                    this.OpMode.telemetry.addData(colorSensor.getDeviceName() + " R, G, B, A", String.format(Locale.US, "%s, %s, %s, %s",
                            colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()));
                    this.OpMode.telemetry.addLine();
                } else if (device instanceof DistanceSensor) {
                    DistanceSensor distanceSensor = (DistanceSensor) device;
                    this.OpMode.telemetry.addData(distanceSensor.getDeviceName() + " distance", distanceSensor.getDistance(DistanceUnit.INCH));
                    this.OpMode.telemetry.addLine();
                }
            }
        }

        if (this.imu != null) {
            if (this.imu.isGyroCalibrated()) {
                Orientation angle = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);
                this.OpMode.telemetry.addData("Robot first angle", angle.firstAngle + " " + angle.angleUnit)
                        .addData("Robot second angle", angle.secondAngle + " " + angle.angleUnit)
                        .addData("Robot third angle", angle.thirdAngle + " " + angle.angleUnit);
                this.OpMode.telemetry.addLine();
            }
        }


        if (this.goldDetector != null) {
            if (this.goldDetector.isFound()) {
                this.OpMode.telemetry.addData("Gold detector", "Found gold");
                this.OpMode.telemetry.addData("Gold location", this.goldDetector.getFoundRect().x + ", " + this.goldDetector.getFoundRect().y);
                this.OpMode.telemetry.addLine();
            } else {
                this.OpMode.telemetry.addData("Gold detector", "Still searching...");
            }
        }


        // Update the telemetry
        this.OpMode.telemetry.update();

    }


    /**
     * Returns if <i>any</i> of the provided motors are within its provided margin of error.
     *
     * @param error  The margin of error its allowed (in encoder ticks).
     * @param motors The motors to check.
     * @return Returns true if any of the motors are within its given margin of error. If all of them are outside the margin of error then it returns false.
     */
    boolean isThere(int error, DcMotor... motors) {
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

        this.status("Starting detector");
        this.goldDetector.enable();

        this.status("Ready!");

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
            this.updateTelemetry();
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


    void armDrive(double speed, int armPos, double timeoutS) {

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

            // Update telemetry as this is running
            this.updateTelemetry();
        }


        this.armMotorR.setPower(0);
        this.armMotorL.setPower(0);

    }


    // TODO: Test
    void turnToDegree(double speed, float turnToAngle, BNO055IMU imu, double timeoutS) {

        double d = 11, currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle,
                circumference = d * 3.14;
        
        int distance = (int) (Math.round(3.14 * (circumference) * (360 - Math.abs(turnToAngle - currentAngle))) / Math.pow(EncoderNumberChangePerInch, 2));

        this.OpMode.telemetry.addData("Distance", distance);
        this.OpMode.telemetry.update();
        this.OpMode.sleep(1000);

        this.encoderDrive(speed, -distance, distance, timeoutS);
    }


    void encoderDrive(double speed, int leftInches, int rightInches, double timeoutS) {
        // Its literally distinctDrive, but with 2 motors
        this.distinctDrive(speed, leftInches, leftInches, rightInches, rightInches, timeoutS);
    }


    void distinctDrive(double speed, int LFInches, int LBInches, int RFInches, int RBInches, double timeoutS) {

        // Reset the motor encoders, and set them to RUN_TO_POSITION
        this.resetMotors(this.left_front, this.left_back, this.right_front, this.right_back);

        // Set the individual drive motor positions
        this.left_front.setTargetPosition(LFInches * EncoderNumberChangePerInch);
        this.right_front.setTargetPosition(RFInches * EncoderNumberChangePerInch);
        this.left_back.setTargetPosition(LBInches * EncoderNumberChangePerInch);
        this.right_back.setTargetPosition(RBInches * EncoderNumberChangePerInch);

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

            this.updateTelemetry();
        }

        // Stop all motion, and reset the motors
        this.resetMotors(this.left_back, this.left_front, this.right_back, this.right_front);
    }
}