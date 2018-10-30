package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
public class config {

    // Field measurements
    public final float mmPerInch = 25.4f;
    public final float mmFTCFieldWidth = (12 * 6) * mmPerInch; // The width of the FTC field (from the center point to the outer panels)
    public final float mmTargetHeight = (6) * mmPerInch; // The height of the center of the target image above the floor

    // DcMotors and servos used on the robot
    public DcMotor left_front, right_front, left_back, right_back;
    public Servo IO_Servo_Left, IO_Servo_Right;


    // Version 2 color sensor
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    // Stuff for vision
    public VuforiaTrackables pictures;
    public VuforiaLocalizer vuforia;
    public VuforiaTrackable BlueRover, RedFootprint, FrontCraters, BackSpace;
    public boolean VisionIsActive = false;
    public String target = "None";
    List<VuforiaTrackable> VisionTargets = new ArrayList<>();

    // Telemetry stuff
    private Telemetry telemetry;

    config(Telemetry t) {
        this.telemetry = t;
    }

    /**
     * Goes through the configureation of the robot, even updating the telemetry :)
     * <br>
     * <br>
     *
     * @param hardware - The HardwareMap of the robot. Just type <code>this.hardwareMap</code> for this parameter.
     */
    public void ConfigureRobtHardware(HardwareMap hardware) {

        // Update telemetry that robot is initializing...
        telemetry.addData("Status", "Initializing robot. Please wait...");
        telemetry.update();

        // Declare and setup left_front
        left_front = hardware.dcMotor.get("left front");
        left_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        left_front.setMode(RunMode.RUN_WITHOUT_ENCODER);
        left_front.setDirection(Direction.FORWARD);

        // Declare and setup right_front
        right_front = hardware.dcMotor.get("right front");
        right_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        right_front.setMode(RunMode.RUN_WITHOUT_ENCODER);
        right_front.setDirection(Direction.REVERSE);

        // Declare and setup left_back
        left_back = hardware.dcMotor.get("left back");
        left_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        left_back.setMode(RunMode.RUN_WITHOUT_ENCODER);
        left_back.setDirection(Direction.FORWARD);

        // Declare and setup right_back
        right_back = hardware.dcMotor.get("right back");
        right_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        right_back.setMode(RunMode.RUN_WITHOUT_ENCODER);
        right_back.setDirection(Direction.REVERSE);

        //Declare and setup for IO
        IO_Servo_Left = hardware.servo.get("IO Servo Left");
        IO_Servo_Right = hardware.servo.get("IO Servo Right");
        /*
        IO_Intake = hardware.dcMotor.get("Intake");
        IO_Intake.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        IO_Intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
        IO_Intake.setDirection(Direction.FORWARD);
        */


        // Update telemetry to signal done!
        telemetry.addData("Status", "Ready!");
        telemetry.update();

    }

    /**
     * Updates the telemetry automatically with the current drive power.
     */
    @SuppressLint("DefaultLocale")
    public void updateTelemetry() {
        telemetry.addData("Left front power", String.format("%.2f", this.left_front.getPower()))
                .addData("Right front power", String.format("%.2f", this.right_front.getPower()))
                .addData("Left back power", String.format("%.2f", this.left_back.getPower()))
                .addData("Right back power", String.format("%.2f", this.right_back.getPower()))
                .addData("","") // Add a space between the drive power and the encoder values
                .addData("Left front target, current location (displacement)", String.format("%s, %s (%s)", this.left_front.getTargetPosition(), this.left_front.getCurrentPosition(), Math.abs(this.left_front.getCurrentPosition() - this.left_front.getTargetPosition())))
                .addData("Right front target, current location (displacement)", String.format("%s, %s (%s)", this.right_front.getTargetPosition(), this.right_front.getCurrentPosition(), Math.abs(this.right_front.getCurrentPosition() - this.right_front.getTargetPosition())))
                .addData("Left back target, current location (displacement)", String.format("%s, %s (%s)", this.left_back.getTargetPosition(), this.left_back.getCurrentPosition(), Math.abs(this.left_back.getCurrentPosition() - this.right_front.getTargetPosition())))
                .addData("Right back target, current location (displacement)", String.format("%s, %s (%s)", this.right_back.getTargetPosition(), this.right_back.getCurrentPosition(), Math.abs(this.right_back.getCurrentPosition() - this.right_back.getTargetPosition())));
        telemetry.update();

    }

    /**
     * Checks all the motors have reached their target positions, withing the discrepancy
     *
     * @param discrepancy -- The number of ticks the current position is allowed to be within in order to qualify it as at target
     * @return -- Whether all motors have reached their targets
     */
    public boolean isAtTarget(int discrepancy) {
        return ((Math.abs(this.left_front.getCurrentPosition() - this.left_front.getTargetPosition()) <= discrepancy &&
                (Math.abs(this.right_front.getCurrentPosition() - this.right_front.getTargetPosition()) <= discrepancy) &&
                (Math.abs(this.left_back.getCurrentPosition() - this.left_back.getTargetPosition()) <= discrepancy) &&
                (Math.abs(this.right_back.getCurrentPosition() - this.right_back.getTargetPosition()) <= discrepancy)));
    }


    /**
     * This function sets up everything you need to run vision. Be sure to call this before calling <code>StartTrackingVisionTargets<code/>.
     *
     * @param hardware -- The HardwareMap of the robot. Just type <code>this.hardwareMap</code> for this parameter.
     */
    public void InitializeVision(HardwareMap hardware) {


        if (left_front != null) {
            telemetry.addData("Left front power", String.format(Locale.US, "%.2f", left_front.getPower()));
        }
        if (right_front != null) {
            telemetry.addData("Right front power", String.format(Locale.US, "%.2f", right_front.getPower()));
        }
        if (left_back != null) {
            telemetry.addData("Left back power", String.format(Locale.US, "%.2f", this.left_back.getPower()));
        }
        if (right_back != null) {
            telemetry.addData("Right back power", String.format(Locale.US, "%.2f", this.right_back.getPower()));
        }

        telemetry.addData("", ""); // Add a space between the drive power and the encoder values

        if (left_front != null) {
            if (left_front.getMode() == RunMode.RUN_USING_ENCODER) {
                telemetry.addData("Left front target, current location (displacement)", String.format("%s, %s (%s)", this.left_front.getTargetPosition(), this.left_front.getCurrentPosition(), Math.abs(this.left_front.getCurrentPosition() - this.left_front.getTargetPosition())));
            }
        }
        if (right_front != null) {
            if (right_front.getMode() == RunMode.RUN_USING_ENCODER) {
                telemetry.addData("Right front target, current location (displacement)", String.format("%s, %s (%s)", this.right_front.getTargetPosition(), this.right_front.getCurrentPosition(), Math.abs(this.right_front.getCurrentPosition() - this.right_front.getTargetPosition())));
            }
        }
        if (left_back != null) {
            if (left_back.getMode() == RunMode.RUN_USING_ENCODER) {
                telemetry.addData("Left back target, current location (displacement)", String.format("%s, %s (%s)", this.left_back.getTargetPosition(), this.left_back.getCurrentPosition(), Math.abs(this.left_back.getCurrentPosition() - this.right_front.getTargetPosition())));
            }
        }
        if (right_back != null) {
            if (right_back.getMode() == RunMode.RUN_USING_ENCODER) {
                telemetry.addData("Right back target, current location (displacement)", String.format("%s, %s (%s)", this.right_back.getTargetPosition(), this.right_back.getCurrentPosition(), Math.abs(this.right_back.getCurrentPosition() - this.right_back.getTargetPosition())));
            }
        }

        telemetry.addData("",""); // Add a space between encoder values and servo values

        if (IO_Servo_Left != null) {
            telemetry.addData("IO Servo Left target position", IO_Servo_Left.getPosition());
        }
        if (IO_Servo_Right != null) {
            telemetry.addData("IO Servo Right target position", IO_Servo_Right.getPosition());
        }

        telemetry.addData("",""); // Add a space between servo values and color sensor stuff

        if (sensorColor != null) {
            telemetry.addData("A", sensorColor.alpha()).addData("R", sensorColor.red()).addData("G", sensorColor.green()).addData("B", sensorColor.blue());
        }

        if (sensorDistance != null) {
            telemetry.addData("Distance (mm)", sensorDistance.getDistance(DistanceUnit.MM));
        }

        telemetry.addData("",""); // Add a space between the color sensor stuff and the vision stuff

        if (VisionIsActive) {
            telemetry.addData("Current visible target", target);
        }

        // Update the telemetry
        telemetry.update();
    }

    /**
     * Starts tracking the vision targets. This is quite taxing on the phone, so be sure to end it as soon as you can
     */
    public void StartTrackingVisionTargets() {
        pictures.activate();
        VisionIsActive = true;
    }

    /**
     * Stops tracking the vision targets
     */
    public void StopTrackingVisionTargets() {
        pictures.deactivate();
        VisionIsActive = false;
    }

}
