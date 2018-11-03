package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        //IO Servo
        double IOLeftServoClose = 0.3,IOLeftServoHalfOpen = 0.7, IOLeftServoOpen = 0.95;
        double IORightServoClose = 0.6, IORightServoHalfOpen = 0.2, IORightServoOpen = 0;
        //MOTORS Power
        double speedForTurn = 0.5, speedForMove =0.6, speedForSide = 0.9;
        double intakePower = 1;
        double armPower =1;
        double extendPower = 0.5;
        // limit position
        int armMaxPosition = 0, armMinPosition = -680;
        int extendMaxPosition = 500, extendMinPosition = 0;
        robot.armMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotorExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //ColorSensor
        final double SCALE_FACTOR = 255;
        float hsvValuesLeft[] = {0F, 0F, 0F};
        float hsvValuesRight[] = {0F, 0F, 0F};
        int leftObject=0, rightObject=0;//0 means nothing, 1 means yellow, 2 means white
        // Wait for the start button to be pressed
        waitForStart();

        // Code that we want to run repeatedly
        while (opModeIsActive()) {

            // Copy paste from Henry's Iterative program, with some formatting changes :)

            // Setup a variable for each drive wheel to save power level for telemetry
            double left1Power, right1Power, left2Power, right2Power, allPower = 1;
            double leftDistance=10,rightDistance=10;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double driveForward = gamepad2.left_stick_y*speedForMove, driveRightSide = gamepad2.left_stick_x*speedForSide;
            double turnRight = -gamepad2.right_stick_x*speedForTurn;
            double armUp;
            double armExtend=0;



            left1Power = Range.clip((-driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
            right1Power = Range.clip((driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;
            left2Power = Range.clip((driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
            right2Power = Range.clip((-driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;
            armUp=(-gamepad1.left_stick_y)*armPower;

            if (gamepad1.left_stick_button)
            {
                armExtend+=extendPower;
            }
            if(gamepad1.right_stick_button)
            {
                armExtend-=extendPower;
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
            if (robot.armMotorL.getCurrentPosition()>= armMaxPosition && armUp>=0)
            {
                armUp=0;
            }
            if(robot.armMotorL.getCurrentPosition()<= armMinPosition && armUp<=0)
            {
                armUp=0;
            }
            robot.armMotorL.setPower(armUp);
            robot.armMotorR.setPower(armUp);

            if(robot.armMotorExtend.getCurrentPosition()>=extendMaxPosition && armExtend>=0)
            {
                armExtend=0;
            }
            if(robot.armMotorExtend.getCurrentPosition()<=extendMinPosition && armExtend<=0)
            {
                armExtend=0;
            }
            robot.armMotorExtend.setPower(armExtend);


            //
            /*
            if (gamepad1.b)
            {
                if (leftObject==1) {
                    robot.IO_Servo_Left.setPosition(IOLeftServoHalfOpen);
                }
                else if(leftObject==2){
                    robot.IO_Servo_Left.setPosition(IOLeftServoOpen);
                }
                else{
                    robot.IO_Servo_Left.setPosition(IOLeftServoClose);
                }
                if (rightObject==1) {
                    robot.IO_Servo_Right.setPosition(IORightServoHalfOpen);
                }
                else if(rightObject==2){
                    robot.IO_Servo_Right.setPosition(IORightServoOpen);
                }
                else{
                    robot.IO_Servo_Right.setPosition(IORightServoClose);
                }
            }
            else{
                robot.IO_Servo_Left.setPosition(IOLeftServoClose);
                robot.IO_Servo_Right.setPosition(IORightServoClose);
            }
            */
            if(gamepad1.a)
            {
                robot.IO_Motor.setPower(intakePower);
                robot.IO_Servo_Left.setPosition(IOLeftServoClose);
                robot.IO_Servo_Right.setPosition(IORightServoClose);
                /*
                Color.RGBToHSV((int) (robot.sensorColorLeft.red() * SCALE_FACTOR),
                        (int) (robot.sensorColorLeft.green() * SCALE_FACTOR),
                        (int) (robot.sensorColorLeft.blue() * SCALE_FACTOR),
                        hsvValuesLeft);//Hue value is hsvValuesLeft[0]
                Color.RGBToHSV((int) (robot.sensorColorRight.red() * SCALE_FACTOR),
                        (int) (robot.sensorColorRight.green() * SCALE_FACTOR),
                        (int) (robot.sensorColorRight.blue() * SCALE_FACTOR),
                        hsvValuesRight);//Hue value is hsvValuesRight[0]

                leftDistance=robot.sensorDistanceLeft.getDistance(DistanceUnit.CM);
                rightDistance=robot.sensorDistanceRight.getDistance(DistanceUnit.CM);
                if (leftDistance<=5.5) {
                    if (hsvValuesLeft[0]>=25&&hsvValuesLeft[0]<=45)
                    {
                        leftObject=1;
                    }
                    else
                    {
                        leftObject=2;
                    }
                }
                else
                {
                    leftObject=0;
                }
                if (rightDistance<=5.5) {
                    if (hsvValuesRight[0] >= 25 && hsvValuesRight[0] <= 45) {
                        rightObject = 1;
                    }
                    else {
                        rightObject = 2;
                    }
                }
                else{
                    rightObject=0;
                }
                */
            }
            else
            {
                robot.IO_Motor.setPower(0);
            }
            if (gamepad1.y)
            {
                robot.IO_Servo_Left.setPosition(IOLeftServoOpen);
                robot.IO_Servo_Right.setPosition(IORightServoOpen);
            }
            if (gamepad1.x)
            {
                robot.IO_Motor.setPower(-intakePower);
            }
            if (gamepad1.b)
            {
                robot.IO_Servo_Right.setPosition(IORightServoHalfOpen);
            }
            // Update telemetry
            //robot.updateTelemetry();
            /*
            telemetry.addData("ObjectL%7d",leftObject);
            telemetry.addData("ObjectR%7d",rightObject);
            telemetry.addData("DistanceL%7d",robot.sensorDistanceLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("DistanceL%7d", leftDistance);
            telemetry.addData("DistanceR%7d",robot.sensorDistanceRight.getDistance(DistanceUnit.CM));
            telemetry.addData("DistanceR%7d",rightDistance);

            telemetry.addData("ArmPosition%7d", robot.armMotorL.getCurrentPosition());
            telemetry.addData("armPower%7d",armUp);
            telemetry.addData("ExtendPosition%7d",robot.armMotorExtend.getCurrentPosition());
            telemetry.addData("extendPower%7d", armExtend);
            */
            telemetry.addData("%7d",gamepad2.right_stick_x);

            telemetry.update();

        }

    }
}
