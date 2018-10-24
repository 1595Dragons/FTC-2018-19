package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    // DcMotors used on the robot
    public DcMotor left_front, right_front, left_back, right_back;
    public Servo IO_Servo_Left, IO_Servo_Right, IO_Servo_Main;
    public DcMotor IO_Intake;
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
        IO_Servo_Main = hardware.servo.get("IO Servo Main");

        IO_Intake = hardware.dcMotor.get("Intake");
        IO_Intake.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        IO_Intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
        IO_Intake.setDirection(Direction.FORWARD);



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

}
