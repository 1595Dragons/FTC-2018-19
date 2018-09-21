package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is the config file for the robot. It also has (or will have) many useful fuctions in it.
 * <p>
 * For example rather than typing: <br>
 * <br>
 * <code>leftDrive = hardware.dcMotor.get("left_drive");<br>
 * rightDrive = hardware.dcMotor.get("right_drive");<br>
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
    public DcMotor leftDrive, rightDrive, leftDrive2, rightDrive2;

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

        // Declare and setup leftDrive
        leftDrive = hardware.dcMotor.get("left_drive");
        leftDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setDirection(Direction.FORWARD);

        // Declare and setup rightDrive
        rightDrive = hardware.dcMotor.get("right_drive");
        rightDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setDirection(Direction.REVERSE);

        // Declare and setup leftDrive2
        leftDrive2 = hardware.dcMotor.get("left_drive2");
        leftDrive2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        leftDrive2.setMode(RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setDirection(Direction.FORWARD);

        // Declare and setup rightDrive2
        rightDrive2 = hardware.dcMotor.get("right_drive2");
        rightDrive2.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightDrive2.setMode(RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setDirection(Direction.REVERSE);

        // Update telemetry to signal done!
        telemetry.addData("Status", "Ready!");
        telemetry.update();

    }

    /**
     * Updates the telemetry automatically with the current drive power.
     */
    @SuppressLint("DefaultLocale")
    public void updateTelemetry() {
        telemetry.addData("Left drive power", String.format("%.2f", this.leftDrive.getPower()))
                .addData("Right drive power", String.format("%.2f", this.rightDrive.getPower()))
                .addData("Left 2 drive power", String.format("%.2f", this.leftDrive2.getPower()))
                .addData("Right 2 drive power", String.format("%.2f", this.rightDrive2.getPower()))
                .addData("Left target, current location (displacement)", String.format("%s, %s (%s)", this.leftDrive.getTargetPosition(), this.leftDrive.getCurrentPosition(), Math.abs(this.leftDrive.getCurrentPosition() - this.leftDrive.getTargetPosition())))
                .addData("Right target, current location (displacement)", String.format("%s, %s (%s)", this.rightDrive.getTargetPosition(), this.rightDrive.getCurrentPosition(), Math.abs(this.rightDrive.getCurrentPosition() - this.rightDrive.getTargetPosition())))
                .addData("Left 2 target, current location (displacement)", String.format("%s, %s (%s)", this.leftDrive2.getTargetPosition(), this.leftDrive2.getCurrentPosition(), Math.abs(this.leftDrive2.getCurrentPosition() - this.rightDrive.getTargetPosition())))
                .addData("Right 2 target, current location (displacement)", String.format("%s, %s (%s)", this.rightDrive2.getTargetPosition(), this.rightDrive2.getCurrentPosition(), Math.abs(this.rightDrive2.getCurrentPosition() - this.rightDrive2.getTargetPosition())));
        telemetry.update();

    }

    /**
     * Checks all the motors have reached their target positions, withing the discrepancy
     *
     * @param discrepancy -- The number of ticks the current position is allowed to be within in order to qualify it as at target
     * @return -- Whether all motors have reached their targets
     */
    public boolean isAtTarget(int discrepancy) {
        return ((Math.abs(this.leftDrive.getCurrentPosition() - this.leftDrive.getTargetPosition()) <= discrepancy &&
                (Math.abs(this.rightDrive.getCurrentPosition() - this.rightDrive.getTargetPosition()) <= discrepancy) &&
                (Math.abs(this.leftDrive2.getCurrentPosition() - this.leftDrive2.getTargetPosition()) <= discrepancy) &&
                (Math.abs(this.rightDrive2.getCurrentPosition() - this.rightDrive2.getTargetPosition()) <= discrepancy)));
    }

}
