package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Size;

import java.util.Locale;


/**
 * Created by Stephen Ogden on 9/13/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
class RobotConfig {


    DcMotor left1, right1, left2, right2, climber, intake, arm;


    int maxClimberPos = 4150, minClimberPos = 0;


    GoldDetector goldDetector;


    private BNO055IMU gyro;


    private Telemetry telemetry;


    RobotConfig(Telemetry t) {
        this.telemetry = t;
    }


    /**
     * Initializes and configures the robot's motors and anything extra that needs to be done before running the main programs.
     *
     * @param hardware The hardware map for the robot.
     */
    void configureRobot(HardwareMap hardware) {

        // Declare and setup left1
        status("Setting up left1");
        left1 = hardware.dcMotor.get("left1");
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Declare and setup right1
        status("Setting up right1");
        right1 = hardware.dcMotor.get("right1");
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setDirection(DcMotorSimple.Direction.FORWARD);

        // Declare and setup left2
        status("Setting up left2");
        left2 = hardware.dcMotor.get("left2");
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Declare and setup right2
        status("Setting up right2");
        right2 = hardware.dcMotor.get("right2");
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Declare and setup the intake
        status("Setting up intake");
        intake = hardware.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Declare and setup the arm
        status("Setting up arm");
        arm = hardware.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        // Declare and setup climber motor
        status("Setting up climber motor");
        climber = hardware.dcMotor.get("climb");
        climber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setDirection(DcMotorSimple.Direction.FORWARD);

        // Declare and setup the gyro
        status("Setting up gyro");
        gyro = hardware.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro = hardware.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        while (!gyro.isGyroCalibrated()) {
            Thread.yield();
        }

        // Update telemetry to signal done!
        status("Ready!");

    }


    /**
     * Goes through each motor provided and resets its encoder.
     * The number of motors can vary.
     *
     * @param motors The motor to have its encoder reset
     */
    void resetMotors(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


    /**
     * Updates the telemetry to display each of the position for all the motors and sensors (if they aren't null that is)
     */
    void updateTelemetry() {

        if (left1 != null) {
            telemetry.addData("Left1 (target)", String.format(Locale.US, "%d (%d)",
                    left1.getCurrentPosition(),
                    left1.getTargetPosition()));
        }

        if (right1 != null) {
            telemetry.addData("Right1 (target)", String.format(Locale.US, "%d (%d)",
                    right1.getCurrentPosition(),
                    right1.getTargetPosition()));
        }

        if (left2 != null) {
            telemetry.addData("Left2 (target)", String.format(Locale.US, "%d (%d)",
                    left2.getCurrentPosition(),
                    left2.getTargetPosition()));
        }

        if (right2 != null) {
            telemetry.addData("Right2 (target)", String.format(Locale.US, "%d (%d)",
                    right2.getCurrentPosition(),
                    right2.getTargetPosition()));
        }

        if (climber != null) {
            telemetry.addData("Climber (target)", String.format(Locale.US, "%d (%d)",
                    climber.getCurrentPosition(),
                    climber.getTargetPosition()));
        }

        if (arm != null) {
            telemetry.addData("Arm (target)", String.format(Locale.US, "%d (%d)",
                    arm.getCurrentPosition(),
                    arm.getTargetPosition()));
        }

        if (intake != null) {
            telemetry.addData("Intake (target)", String.format(Locale.US, "%d (%d)",
                    intake.getCurrentPosition(),
                    intake.getTargetPosition()));
        }

        if (goldDetector != null) {
            telemetry.addLine();
            if (goldDetector.isFound()) {
                telemetry.addData("Gold detector", String.format(Locale.US, "Found gold at %.3f, %.3f (in terms of center)",
                        Math.abs(goldDetector.getScreenPosition().x - (goldDetector.getInitSize().width / 2)),
                        Math.abs(goldDetector.getScreenPosition().y - (goldDetector.getInitSize().height / 2))));
            } else {
                telemetry.addData("Gold detector", "Still searching...");
            }
        }

        if (gyro != null) {
            telemetry.addLine();
            if (gyro.isGyroCalibrated()) {
                telemetry.addData("Gyro angles", String.format(Locale.US, "%.3f%s X, %.3f%s Y, %.3f%s Z",
                        getAngle().firstAngle, getAngle().angleUnit.name().toLowerCase(),
                        getAngle().secondAngle, getAngle().angleUnit.name().toLowerCase(),
                        getAngle().thirdAngle, getAngle().angleUnit.name().toLowerCase()));
            } else {
                telemetry.addData("Gyro error", "Gyro isn't calibrated");
            }

            if (gyro.isAccelerometerCalibrated()) {
                telemetry.addData("Gyro position", String.format(Locale.US, "%.3f%s X, %.3f%s Y, %.3f%s Z",
                        gyro.getPosition().x, gyro.getPosition().unit.toString(),
                        gyro.getPosition().y, gyro.getPosition().unit.toString(),
                        gyro.getPosition().z, gyro.getPosition().unit.toString()));
                telemetry.addData("Gyro velocity", String.format(Locale.US, "%.3f%s X, %.3f%s Y, %.3f%s Z",
                        gyro.getVelocity().xVeloc, gyro.getVelocity().unit.toString(),
                        gyro.getVelocity().yVeloc, gyro.getVelocity().unit.toString(),
                        gyro.getVelocity().zVeloc, gyro.getVelocity().unit.toString()));
                telemetry.addData("Gyro acceleration", String.format(Locale.US, "%.3f%s X, %.3f%s Y, %.3f%s Z",
                        gyro.getAcceleration().xAccel, gyro.getAcceleration().unit.toString(),
                        gyro.getAcceleration().yAccel, gyro.getAcceleration().unit.toString(),
                        gyro.getAcceleration().zAccel, gyro.getAcceleration().unit.toString()));
            } else {
                telemetry.addData("Gyro error", "Accelerometer isn't calibrated!");
            }
        }

        telemetry.update();
    }


    /**
     * Returns if the provided motor is within its provided margin of error.
     *
     * @param error The margin of error its allowed (in encoder ticks).
     * @param motor The motor to check.
     * @return Returns true if the motor is within its given margin of error. If it's outside its margin of error then it returns false.
     */
    boolean isThere(int error, DcMotor motor) {
        int delta = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        return delta <= error;
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
     * Returns if the provided angle on the given axis is within the provided margin of error.
     *
     * @param error  The margin of error allowed in degrees.
     * @param degree The degrees trying to be reached.
     * @param axis   The axis to get the reading from.
     * @return Returns true if the provided angle on the given axis is within the provided margin of error. If it's outside its margin error it returns false.
     */
    boolean isThere(int error, double degree, org.firstinspires.ftc.teamcode.Axis axis) {
        boolean reached = false;
        double angle = 0;
        switch (axis) {
            case X:
                angle = getAngle().firstAngle;
                break;
            case Y:
                angle = getAngle().secondAngle;
                break;
            case Z:
                angle = getAngle().thirdAngle;
                break;
        }

        double delta = Math.abs(degree - angle);
        return delta <= error;

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
        goldDetector.init(hardware.appContext, CameraViewDisplay.getInstance(), 1, false);
        goldDetector.useDefaults();

        // Apply the score based on color
        status("Applying color deviation scoring");
        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.COLOR_DEVIATION;

        status("Ready!");

    }


    /**
     * A small helper function that updates the telemetry status with the provided string.
     *
     * @param string The description for the status.
     */
    void status(String string) {
        telemetry.addData("Status", string);
        telemetry.update();
    }


    /**
     * @param direction
     * @param inches
     * @param maxPower
     */
    void driveDistance(MecanumDriveDirection direction, int inches, double maxPower) {

        // Top secret bleeding edge shit right here
        // 1700 (ticks)/(rot motor) * 1/(π*4) (rot wheel/in) = 135 ticks/in
        final int ticksPerRotation = 1700;
        final double wheelRotationPerInch = (1 / (Math.PI * 4));
        final double drive_equation = ticksPerRotation * wheelRotationPerInch;


        int ticks = (int) Math.round(inches * drive_equation);
        // TODO: Because of vector math, the total number of ticks the wheels need to go it going to be different depending on the direction
        // This only really applies to all but forward and backwards
        switch (direction) {
            case BACKWARD:
                left1.setTargetPosition(ticks);
                left2.setTargetPosition(ticks);
                right1.setTargetPosition(ticks);
                right2.setTargetPosition(ticks);
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case FORWARD:
                left1.setTargetPosition(-1 * ticks);
                left2.setTargetPosition(-1 * ticks);
                right1.setTargetPosition(-1 * ticks);
                right2.setTargetPosition(-1 * ticks);
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case LEFT:
                left1.setTargetPosition((int) (Math.round(1.35 * ticks)));
                left2.setTargetPosition((int) (Math.round(-1.35 * ticks)));
                right1.setTargetPosition((int) (Math.round(-1.35 * ticks)));
                right2.setTargetPosition((int) (Math.round(1.35 * ticks)));
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case RIGHT:
                left1.setTargetPosition((int) (Math.round(-1.275 * ticks)));
                left2.setTargetPosition((int) (Math.round(1.275 * ticks)));
                right1.setTargetPosition((int) (Math.round(1.275 * ticks)));
                right2.setTargetPosition((int) (Math.round(-1.275 * ticks)));
                setMaxPower(maxPower, left1, right1, left2, right2);
                break;
            case DIAGUPLEFT:
                left1.setTargetPosition((int) (Math.round(1.15 * ticks)));
                right2.setTargetPosition((int) (Math.round(1.15 * ticks)));
                setMaxPower(maxPower, left1, right2);
                break;
            case DIAGDOWNRIGHT:
                left1.setTargetPosition((int) (Math.round(-1.15 * ticks)));
                right2.setTargetPosition((int) (Math.round(-1.15 * ticks)));
                setMaxPower(maxPower, left1, right2);
                break;
            case DIAGUPRIGHT:
                left2.setTargetPosition((int) (Math.round(1.15 * ticks)));
                right1.setTargetPosition((int) (Math.round(1.15 * ticks)));
                setMaxPower(maxPower, left2, right1);
                break;
            case DIAGDOWNLEFT:
                left2.setTargetPosition((int) (Math.round(-1.15 * ticks)));
                right1.setTargetPosition((int) (Math.round(-1.15 * ticks)));
                setMaxPower(maxPower, left2, right1);
                break;
        }

    }

    /**
     * @param degree
     * @param maxPower
     */
    void driveToDegree(int degree, int maxPower) {
        double error = degree - getAngle().thirdAngle; // First angle is X, second is Y, and third angle is Z
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        double steer = Range.clip(error * 0.15, -1, 1); // A weird PID

        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double leftSpeed = maxPower - steer, rightSpeed = maxPower + steer;

        // Normalize speeds if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > maxPower) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        left1.setPower(leftSpeed);
        left2.setPower(leftSpeed);
        right1.setPower(rightSpeed);
        right2.setPower(rightSpeed);

    }

    /**
     * @return Returns the angles read by the gyro.
     */
    private Orientation getAngle() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }


    /**
     * Helper function mainly used by {@link #driveDistance(MecanumDriveDirection, int, double)} in order to set the maximum allowed power for each provided motor.
     *
     * @param power  The maximum power output value (from 0 to 1).
     * @param motors The motors this applies to.
     */
    void setMaxPower(double power, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

}