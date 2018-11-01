package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

/**
 * Created by Stephen Ogden on 9/20/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@TeleOp(name="Mechanum OpMode Linear", group="Test")
public class mechanum_OpMode_Linear extends LinearOpMode {

    // Declare the config file, that way we can use the pre-made fictions for cleaner code
    private config robot = new config(this.telemetry);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware(this.hardwareMap);

        double IOLeftServoClose = 0.1,IOLeftServoOpen = 0.05;
        double IORightServoClose = 0.8, IORightServoOpen = 0.9;

        double IOLeftServoPosition = IOLeftServoOpen ,IORightServoPosition = IORightServoOpen;

        double speedForTurn = 0.55, speedForMove =0.65, speedForSide = 0.9;
        // Wait for the start button to be pressed
        waitForStart();

        // Code that we want to run repeatedly
        while (opModeIsActive()) {

            // Copy paste from Henry's Iterative program, with some formatting changes :)

            // Setup a variable for each drive wheel to save power level for telemetry
            double left1Power, right1Power, left2Power, right2Power, allPower = 1;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double driveForward = gamepad1.left_stick_y*speedForMove, driveRightSide = gamepad1.left_stick_x*speedForSide;
            double turnRight = -gamepad1.right_stick_x*speedForTurn;


            left1Power = Range.clip((-driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
            right1Power = Range.clip((driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;
            left2Power = Range.clip((driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
            right2Power = Range.clip((-driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;

            if(gamepad1.a )//A intake
            {
                IOLeftServoPosition=IOLeftServoClose;
                IORightServoPosition=IORightServoClose;
            }
            if(gamepad1.b)//B outtake
            {
                IOLeftServoPosition=IOLeftServoOpen;
                IORightServoPosition=IORightServoOpen;
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.left_front.setPower(left1Power);
            robot.right_front.setPower(right1Power);
            robot.left_back.setPower(left2Power);
            robot.right_back.setPower(right2Power);

            //
            robot.IO_Servo_Left.setPosition(IOLeftServoPosition);
            robot.IO_Servo_Right.setPosition(IORightServoPosition);

            // Update telemetry
            //robot.updateTelemetry();
            if (robot.left_front != null) {
                telemetry.addData("Left front power", String.format(Locale.US, "%.2f", robot.left_front.getPower()));
            }
            if (robot.right_front != null) {
                telemetry.addData("Right front power", String.format(Locale.US, "%.2f", robot.right_front.getPower()));
            }
            if (robot.left_back != null) {
                telemetry.addData("Left back power", String.format(Locale.US, "%.2f", robot.left_back.getPower()));
            }
            if (robot.right_back != null) {
                telemetry.addData("Right back power", String.format(Locale.US, "%.2f", robot.right_back.getPower()));
            }
            telemetry.addData("Forward (ls Y)", String.format("%.2f", gamepad1.left_stick_y))
                    .addData("Sideway (ls X)", String.format("%.2f", gamepad1.left_stick_x))
                    .addData("Turn (rs X)", String.format("%.2f", gamepad1.right_stick_x));
            telemetry.update();

        }

    }
}
