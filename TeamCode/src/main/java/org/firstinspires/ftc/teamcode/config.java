package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

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
 * <p>
 * FTC 6128 | 7935
 * FRC 1595
 */
class config {

    // 28 (ticks)/(rot motor) * 49 (rot motor/rot wheel) * 1/(3.14*4) (rot wheel/in) = 109 ticks/in

    private final int ticksPerRotation = 1700;
    private final double whellRotationPerInch = (1 / (Math.PI * 4));
    private final double drive_equation = ticksPerRotation * whellRotationPerInch;

    DcMotor left1, right1, left2, right2, climber, intake, arm;

    int maxClimberPos = 10000, minClimberPos = 0;

    private Telemetry telemetry;

    config(Telemetry t) {
        this.telemetry = t;
    }

    void ConfigureRobot(HardwareMap hardware) {

        // Declare and setup left1
        status("Setting up left1");
        left1 = hardware.dcMotor.get("left1");
        left1.setZeroPowerBehavior(BRAKE);
        left1.setMode(RUN_USING_ENCODER);
        left1.setDirection(REVERSE);

        // Declare and setup right1
        status("Setting up right1");
        right1 = hardware.dcMotor.get("right1");
        right1.setZeroPowerBehavior(BRAKE);
        right1.setMode(RUN_USING_ENCODER);
        right1.setDirection(FORWARD);

        // Declare and setup left2
        status("Setting up left2");
        left2 = hardware.dcMotor.get("left2");
        left2.setZeroPowerBehavior(BRAKE);
        left2.setMode(RUN_USING_ENCODER);
        left2.setDirection(REVERSE);

        // Declare and setup right2
        status("Setting up right2");
        right2 = hardware.dcMotor.get("right2");
        right2.setZeroPowerBehavior(BRAKE);
        right2.setMode(RUN_USING_ENCODER);
        right2.setDirection(FORWARD);

        status("Setting up intake");
        intake = hardware.dcMotor.get("intake");
        intake.setZeroPowerBehavior(BRAKE);
        intake.setMode(RUN_USING_ENCODER);
        intake.setDirection(FORWARD);

        status("Setting up arm");
        arm = hardware.dcMotor.get("arm");
        arm.setZeroPowerBehavior(BRAKE);
        arm.setMode(RUN_USING_ENCODER);
        arm.setDirection(FORWARD);

        // Declare and setup climber motor
        status("Setting up climber motor");
        climber = hardware.dcMotor.get("climb");
        climber.setZeroPowerBehavior(BRAKE);
        climber.setMode(STOP_AND_RESET_ENCODER);
        climber.setMode(RUN_USING_ENCODER);
        climber.setDirection(FORWARD);

        // Update telemetry to signal done!
        status("Ready!");

    }

    void setupForAuto() {
        left1.setMode(STOP_AND_RESET_ENCODER);
        left2.setMode(STOP_AND_RESET_ENCODER);
        right1.setMode(STOP_AND_RESET_ENCODER);
        right2.setMode(STOP_AND_RESET_ENCODER);
        climber.setMode(STOP_AND_RESET_ENCODER);
        left1.setMode(RUN_TO_POSITION);
        left2.setMode(RUN_TO_POSITION);
        right1.setMode(RUN_TO_POSITION);
        right2.setMode(RUN_TO_POSITION);
        climber.setMode(RUN_TO_POSITION);
    }

    void updateTelemetry() {

        if (left1 != null) {
            telemetry.addData("Left1 (target)", String.format(Locale.US, "%d (%d)", left1.getCurrentPosition(), left1.getTargetPosition()));
        }

        if (right1 != null) {
            telemetry.addData("Right1 (target)", String.format(Locale.US, "%d (%d)", right1.getCurrentPosition(), right1.getTargetPosition()));
        }

        if (left2 != null) {
            telemetry.addData("Left2 (target)", String.format(Locale.US, "%d (%d)", left2.getCurrentPosition(), left2.getTargetPosition()));
        }

        if (right2 != null) {
            telemetry.addData("Right2 (target)", String.format(Locale.US, "%d (%d)", right2.getCurrentPosition(), right2.getTargetPosition()));
        }

        if (climber != null) {
            telemetry.addData("Climber (target)", String.format(Locale.US, "%d (%d)", climber.getCurrentPosition(), climber.getTargetPosition()));
        }

        if (arm != null) {
            telemetry.addData("Arm (target)", String.format(Locale.US, "%d (%d)", arm.getCurrentPosition(), arm.getTargetPosition()));
        }

        if (intake != null) {
            telemetry.addData("Intake (target)", String.format(Locale.US, "%d (%d)", intake.getCurrentPosition(), intake.getTargetPosition()));
        }

        telemetry.update();
    }

    boolean isThere(int error, DcMotor motor) {
        int delta = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        return delta <= error;
    }

    // Returns if the destination was reached
    void driveDistance(MecanumDriveDirection direction, int inches, double maxPower) {
        int ticks = (int) Math.round(inches * drive_equation);
        // TODO: Because of vector math, the total number of ticks the wheels need to go it going to be different depending on the direction
        // This only really applies to all but forward and backwards
        switch (direction) {
            case FORWARD:
                left1.setTargetPosition(ticks);
                left2.setTargetPosition(ticks);
                right1.setTargetPosition(ticks);
                right2.setTargetPosition(ticks);
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case BACKWARD:
                left1.setTargetPosition(-1 * ticks);
                left2.setTargetPosition(-1 * ticks);
                right1.setTargetPosition(-1 * ticks);
                right2.setTargetPosition(-1 * ticks);
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case RIGHT:
                left1.setTargetPosition((int) (Math.round(1.2 * ticks)));
                left2.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                right1.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                right2.setTargetPosition((int) (Math.round(1.2 * ticks)));
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case LEFT:
                left1.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                left2.setTargetPosition((int) (Math.round(1.2 * ticks)));
                right1.setTargetPosition((int) (Math.round(1.2 * ticks)));
                right2.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case DIAGUPLEFT:
                left1.setTargetPosition((int) (Math.round(1.2 * ticks)));
                right2.setTargetPosition((int) (Math.round(1.2 * ticks)));
                setMaxPower(maxPower, left1, right2);
                break;
            case DIAGDOWNRIGHT:
                left1.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                right2.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                setMaxPower(maxPower, left1, right2);
                break;
            case DIAGUPRIGHT:
                left2.setTargetPosition((int) (Math.round(1.2 * ticks)));
                right1.setTargetPosition((int) (Math.round(1.2 * ticks)));
                setMaxPower(maxPower, left2, right1);
                break;
            case DIAGDOWNLEFT:
                left2.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                right1.setTargetPosition((int) (Math.round(-1.2 * ticks)));
                setMaxPower(maxPower, left2, right1);
                break;
        }

    }

    private void status(String string) {
        telemetry.addData("Status", string);
        telemetry.update();
    }

    void setMaxPower(double power, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    static int BooleanToInt(boolean bool) {
        return bool ? 1 : 0;
    }

    static boolean IntToBoolean(int i) {
        return i == 1;
    }

}