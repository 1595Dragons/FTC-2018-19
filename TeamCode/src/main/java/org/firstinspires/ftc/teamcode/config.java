package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * This is the config file for the robot. It also has (or will have) many useful functions in it.
 * <p>
 * For example rather than typing: <br>
 * <br>
 * <code>left_front = hardware.dcMotor.get("left_drive");<br>
 * right_front = hardware.dcMotor.get("right_drive");<br>
 * ...</code>
 * <br>
 * <br>
 * you can just type: <br>
 * <br>
 * <code>private config robot = new config();<br>
 * robot.ConfigureRobotHardware(this.hardwareMap);</code>
 * <br>
 * <br>
 * That's literally the configuration process done!
 * <br>
 * <br>
 * <br>
 * <p>
 * <p>
 * Created by Stephen Ogden on 9/13/18.
 * Modified on 9/20/18.
 * <p>
 * FTC 6128 | 7935
 * FRC 1595
 */
class config {

    final int leftarmUp = 660, leftArmDown = 0, rightArmUp = 660, rightArmDown = 0;
    // Field measurements
    private final float mmPerInch = 25.4f;
    private final float mmFTCFieldWidth = (12 * 6) * mmPerInch; // The width of the FTC field (from the center point to the outer panels)
    private final float mmTargetHeight = (6) * mmPerInch; // The height of the center of the target image above the floor
    GoldDetector goldDetector;
    String target = "None";
    List<VuforiaTrackable> VisionTargets = new ArrayList<>();
    // DcMotors and servos used on the robot
    DcMotor left_front, right_front, left_back, right_back, IO_Motor, armMotorL, armMotorR, armMotorExtend;
    Servo IO_Servo_Left, IO_Servo_Right;
    // Version 2 color sensor

    ColorSensor sensorColorLeft, sensorColorRight;
    DistanceSensor sensorDistanceLeft, sensorDistanceRight;
    private boolean VisionIsActive = false;
    private Dogeforia vuforia;
    private VuforiaTrackables pictures;
    // Telemetry stuff
    private Telemetry telemetry;

    private ElapsedTime time = new ElapsedTime();


    config(Telemetry t) {
        this.telemetry = t;
    }

    /**
     * Goes through the configuration of the robot, even updating the telemetry :)
     * <br>
     * <br>
     *
     * @param hardware - The HardwareMap of the robot. Just type <code>this.hardwareMap</code> for this parameter.
     */
    void ConfigureRobtHardware(HardwareMap hardware) {

        // Declare and setup left_front
        status("Configuring left front motor");
        left_front = hardware.dcMotor.get("left front");
        left_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        left_front.setMode(RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(RunMode.RUN_USING_ENCODER);
        left_front.setDirection(Direction.FORWARD);

        // Declare and setup right_front
        status("Configuring right front motor");
        right_front = hardware.dcMotor.get("right front");
        right_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        right_front.setMode(RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(RunMode.RUN_USING_ENCODER);
        right_front.setDirection(Direction.REVERSE);

        // Declare and setup left_back
        status("Configuring left back motor");
        left_back = hardware.dcMotor.get("left back");
        left_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        left_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(RunMode.RUN_USING_ENCODER);
        left_back.setDirection(Direction.FORWARD);

        // Declare and setup right_back
        status("Configuring right back motor");
        right_back = hardware.dcMotor.get("right back");
        right_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        right_back.setMode(RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(RunMode.RUN_USING_ENCODER);
        right_back.setDirection(Direction.REVERSE);

        //Declare and setup armMotorL and armMotorR
        status("Configuring left arm");
        armMotorL = hardware.dcMotor.get("arm motor left");
        armMotorL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        armMotorL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armMotorL.setMode(RunMode.RUN_USING_ENCODER);
        armMotorL.setDirection(Direction.REVERSE);

        status("Configuring right arm");
        armMotorR = hardware.dcMotor.get("arm motor right");
        armMotorR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        armMotorR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armMotorR.setMode(RunMode.RUN_USING_ENCODER);
        armMotorR.setDirection(Direction.FORWARD);

        status("Configuring arm extender");
        armMotorExtend = hardware.dcMotor.get("arm motor extend");
        armMotorExtend.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        armMotorExtend.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armMotorExtend.setMode(RunMode.RUN_USING_ENCODER);
        armMotorExtend.setDirection(Direction.FORWARD);

        // Declare the left servo for the intake
        status("Setting up left servo");
        IO_Servo_Left = hardware.servo.get("IO Servo Left");

        // Declare the right servo for the intake
        status("Setting up right servo");
        IO_Servo_Right = hardware.servo.get("IO Servo Right");

        //Declare and setup Intake Motor
        status("Configuring Intake Motor");
        IO_Motor = hardware.dcMotor.get("IO motor");
        IO_Motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        IO_Motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        IO_Motor.setMode(RunMode.RUN_USING_ENCODER);
        IO_Motor.setDirection(Direction.REVERSE);

        status("Setting up color sensor");
        sensorColorLeft = hardware.colorSensor.get("color sensor left");
        sensorDistanceLeft = hardware.get(DistanceSensor.class, "color sensor left");
        sensorColorRight = hardware.colorSensor.get("color sensor right");
        sensorDistanceRight = hardware.get(DistanceSensor.class, "color sensor right");

        // Update telemetry to signal done!
        status("Done!");
    }


    void resetMotors(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(RunMode.RUN_TO_POSITION);
        }
    }

    /**
     * Updates the telemetry automatically the appropriate values (basically items that are not null)
     */
    void updateTelemetry() {

        if (left_front != null) {
            telemetry.addData("Left front power", String.format(Locale.US, "%.2f", left_front.getPower()));
        }


        if (right_front != null) {
            telemetry.addData("Right front power", String.format(Locale.US, "%.2f", right_front.getPower()));
        }


        if (left_back != null) {
            telemetry.addData("Left back power", String.format(Locale.US, "%.2f", left_back.getPower()));
        }


        if (right_back != null) {
            telemetry.addData("Right back power", String.format(Locale.US, "%.2f", right_back.getPower()));
        }


        if (IO_Motor != null) {
            telemetry.addData("Intake motor power", String.format(Locale.US, "%.2f", IO_Motor.getPower()));
        }


        if (armMotorL != null) {
            telemetry.addData("Left arm motor power", String.format(Locale.US, "%.2f", armMotorL.getPower()));
        }


        if (armMotorR != null) {
            telemetry.addData("Right arm motor power", String.format(Locale.US, "%.2f", armMotorR.getPower()));
        }


        if (armMotorExtend != null) {
            telemetry.addData("Arm extension motor power", String.format(Locale.US, "%.2f", armMotorExtend.getPower()));
        }

        telemetry.addLine(); // Add a space between the drive power and the encoder values


        if (left_front != null) {
            telemetry.addData("Left front current location", left_front.getCurrentPosition())
                    .addData("Left front target location", left_front.getTargetPosition())
                    .addData("Left front displacement", Math.abs(left_front.getCurrentPosition() - left_front.getTargetPosition()));

        }


        if (right_front != null) {
            telemetry.addData("Right front current location", right_front.getCurrentPosition())
                    .addData("Right front target location", right_front.getTargetPosition())
                    .addData("Right front displacement", Math.abs(right_front.getCurrentPosition() - right_front.getTargetPosition()));
        }


        if (left_back != null) {
            telemetry.addData("Left back current location", left_back.getCurrentPosition())
                    .addData("Left back target location", left_back.getTargetPosition())
                    .addData("Left back displacement", Math.abs(left_back.getCurrentPosition() - left_back.getTargetPosition()));
        }


        if (right_back != null) {
            telemetry.addData("Right back current location", right_back.getCurrentPosition())
                    .addData("Right back target location", right_back.getTargetPosition())
                    .addData("Right back displacement", Math.abs(right_back.getCurrentPosition() - right_back.getTargetPosition()));
        }


        if (IO_Motor != null) {
            telemetry.addData("Intake motor current location", IO_Motor.getCurrentPosition())
                    .addData("Intake motor target location", IO_Motor.getTargetPosition())
                    .addData("Intake motor displacement", Math.abs(IO_Motor.getCurrentPosition() - IO_Motor.getTargetPosition()));
        }


        if (armMotorL != null) {
            telemetry.addData("Left arm current location", armMotorL.getCurrentPosition())
                    .addData("Left arm target location", armMotorL.getTargetPosition())
                    .addData("Left arm displacement", Math.abs(armMotorL.getCurrentPosition() - armMotorL.getTargetPosition()));
        }


        if (armMotorR != null) {
            telemetry.addData("Right arm current location", armMotorR.getCurrentPosition())
                    .addData("Right arm target location", armMotorR.getTargetPosition())
                    .addData("Right arm displacement", Math.abs(armMotorR.getCurrentPosition() - armMotorR.getTargetPosition()));
        }


        if (armMotorExtend != null) {
            telemetry.addData("Arm extension motor current location", armMotorExtend.getCurrentPosition())
                    .addData("Arm extension motor target location", armMotorExtend.getTargetPosition())
                    .addData("Arm extension motor displacement", Math.abs(armMotorExtend.getCurrentPosition() - armMotorExtend.getTargetPosition()));
        }


        telemetry.addLine(); // Add a space between encoder values and servo values


        if (IO_Servo_Left != null) {
            telemetry.addData("IO Servo Left target position", IO_Servo_Left.getPosition());
        }


        if (IO_Servo_Right != null) {
            telemetry.addData("IO Servo Right target position", IO_Servo_Right.getPosition());
        }


        telemetry.addLine(); // Add a space between servo values and color sensor stuff

        if (sensorColorRight != null) {
            telemetry.addData("Right color sensor R, G, B", String.format(Locale.US, "%d, %d, %d", sensorColorRight.red(), sensorColorRight.green(), sensorColorRight.blue()));
        }


        if (sensorColorLeft != null) {
            telemetry.addData("Left color sensor R, G, B", String.format(Locale.US, "%d, %d, %d", sensorColorLeft.red(), sensorColorLeft.green(), sensorColorLeft.blue()));
        }


        if (VisionIsActive) {
            telemetry.addLine(); // Add a space between the color sensor stuff and the vision stuff
            telemetry.addData("Current visible target", target);

        }

        if (goldDetector != null) {
            if (goldDetector.isFound()) {
                telemetry.addData("Vision", "Found gold");
            } else {
                telemetry.addData("Vision", "Still searching...");
            }
        }


        // Update the telemetry
        telemetry.update();

    }

    /**
     * Checks all the motors have reached their target positions, withing the discrepancy
     *
     * @param discrepancy -- The number of ticks the current position is allowed to be within in order to qualify it as at target
     * @return -- Whether all motors have reached their targets
     */
    @Deprecated
    boolean isAtTarget(int discrepancy) {
        return ((Math.abs(this.left_front.getCurrentPosition() - this.left_front.getTargetPosition()) <= discrepancy &&
                (Math.abs(this.right_front.getCurrentPosition() - this.right_front.getTargetPosition()) <= discrepancy) &&
                (Math.abs(this.left_back.getCurrentPosition() - this.left_back.getTargetPosition()) <= discrepancy) &&
                (Math.abs(this.right_back.getCurrentPosition() - this.right_back.getTargetPosition()) <= discrepancy)));
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
     * This function sets up everything you need to run vision. Be sure to call this before calling <code>StartTrackingVisionTargets<code/>.
     *
     * @param hardware -- The HardwareMap of the robot. Just type <code>this.hardwareMap</code> for this parameter.
     */
@Deprecated
    void InitializeVision(HardwareMap hardware) {

        status("Setting up vision system");

        // Get the camera monitor id for the app
        status("Getting the Webcam name");
        WebcamName webcamName = hardware.get(WebcamName.class, "Webcam");


        // Create a variable for passing parameters, such as the key for vuforia, and what camera we want to use (Back vs Selfie camera)
        status("Creating parameters");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(hardware.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardware.appContext.getPackageName()));
        parameters.vuforiaLicenseKey = "AZCmJPP/////AAABmRv4/TqKsEL5ibXvaun2fEQKEUkwXTrdsCcIueZmlFVNNUuH0X6UhMExuB/m/NXRcHP+Wq/W9XTSqiS21EZ/itywzp7hwhlDorTrimz/dFGqKtpA8xI+hoktwHhVbqZkgt+BrZTqYh6jB6VsIUZ/u6xH7eA3mi2AZgoKQrhjGlJt+I0vU/Ge6BH44QQqs/mYW0em2diCwdTXIotik0DvFqW8xlCt3LGjJti23oYcPRIwBx3tHhZb9eZHRaILghHoCsdnBK4fHM0Gl2e/QXbbnq/FeB5mislfemylT7YVHDmTgPgDGFZb1aorTiFm6fQCWX7duJ8YtOzsV2WEUbH64DSHZUYVmhR0rK+IcUbLs9G/";
        parameters.fillCameraMonitorViewParent = true;
        parameters.cameraName = webcamName;

        // Create the pictures engine and pass in the provided parameters
        status("Applying parameters");
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();


        // Load the pictures targets from the engine, that way we can track them
        status("Loading assets");
        pictures = vuforia.loadTrackablesFromAsset("RoverRuckus");

        // Setup the images one by one
        VuforiaTrackable blueRover = pictures.get(0);
        blueRover.setName("Blue rover");

        VuforiaTrackable redFootprint = pictures.get(1);
        redFootprint.setName("Red footprint");

        VuforiaTrackable frontCraters = pictures.get(2);
        frontCraters.setName("Front craters");

        VuforiaTrackable backSpace = pictures.get(3);
        backSpace.setName("Back space");


        // Add all the target to a list, and then activate the tracker
        status("Activating picture targets");
        VisionTargets.addAll(pictures);
        pictures.activate();


        // Create the gold detector
        status("Creating gold detector");
        goldDetector = new GoldDetector();
        ;
        goldDetector.init(hardware.appContext, CameraViewDisplay.getInstance(), 0, true);
        goldDetector.useDefaults();


        // Apply the score based on color
        status("Applying max area scoring");
        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        goldDetector.downscale = 0.8;

        // Set the detector to vuforia
        status("Adding goldDetector to vuforia");
        vuforia.setDogeCVDetector(goldDetector);

        status("Ready!");
    }

    /**
     * Starts tracking the vision targets. This is quite taxing on the phone, so be sure to end it as soon as you can
     */
    @Deprecated
    void StartTrackingVisionTargets() {
        VisionIsActive = true;
        vuforia.showDebug();
        vuforia.enableDogeCV();
        vuforia.enableTrack();
        vuforia.start();
    }

    /**
     * Stops tracking the vision targets
     */
    void StopTrackingVisionTargets() {
        pictures.deactivate();
        VisionIsActive = false;
        vuforia.disableTrack();
        vuforia.disableDogeCV();
        vuforia.stop();
    }


    /**
     * Sets up the vision system (DogeCV) for detecting the gold (piss yellow) cube
     *
     * @param hardware The hardware map of the robot (for getting the app context... android is weird)
     */
    void setupGoldDetector(HardwareMap hardware) {

        status("Creating gold detector");
        goldDetector = new GoldDetector();

        // Set the size of the camera
        status("Setting camera size");
        goldDetector.setAdjustedSize(new Size(270, 480));

        // Init the detector (try to use the defaults)
        status("Applying settings");
        goldDetector.init(hardware.appContext, CameraViewDisplay.getInstance(), 0, false);
        goldDetector.useDefaults();

        // Apply the score based on color
        status("Applying color deviation scoring");
        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.COLOR_DEVIATION;

        status("Starting detector");
        goldDetector.enable();

        status("Ready!");

    }

    boolean searchForGold(int milliseconds) {
        time.reset();
        while (time.milliseconds() < milliseconds) {
            telemetry.addData("Elapsed time", time.milliseconds());
            updateTelemetry();
            if (goldDetector.isFound()) {
                return true;
            }
            Thread.yield();
        }
        return false;
    }

    void status(String string) {
        telemetry.addData("Status", string);
        telemetry.update();
    }

}