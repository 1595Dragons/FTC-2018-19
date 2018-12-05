package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Stephen Ogden on 9/20/18.
 * edited by Henry Xu.
 * FTC 7935
 * FRC 1595
 */


@TeleOp(name="7935 TeleOp Cubed", group="Official")
public class TeleopCubed extends LinearOpMode {

    // Declare the Config file, that way we can use the pre-made fictions for cleaner code
    private Config robot = new Config(this);

    public void runOpMode() {

        // Initialize the robot
        robot.ConfigureRobtHardware(false);

        //IO Servo
        //Servo 位置
        double IOLeftServoClose = 0.2,IOLeftServoHalfOpen = 0.6, IOLeftServoOpen = 0.80;
        double IORightServoClose = 0.7, IORightServoHalfOpen = 0.35, IORightServoOpen = 0.15;
        //MOTORS Power
        double speedForTurn = 0.4, speedForMove =0.5, speedForSide = 0.7;
        double intakePower = 1;
        double armPower =1;
        double extendPower = 0.8;
        // limit position
        int armPositionInitial=0;
        int armMaxPosition = 0, armMinPosition = -680;
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

            double driveForward = gamepad2.left_stick_y*gamepad2.left_stick_y*gamepad2.left_stick_y*speedForMove;
            double driveRightSide = gamepad2.left_stick_x*gamepad2.left_stick_x*gamepad2.left_stick_x*speedForSide;
            double turnRight = -gamepad2.right_stick_x*gamepad2.right_stick_x*gamepad2.right_stick_x*speedForTurn;
            double armUp;
            double armExtend=0;
            left1Power=0;
            left2Power=0;
            right1Power=0;
            right2Power=0;
            //prevent small input from stick
            if (driveForward>=-0.1&&driveForward<=0.1){
                driveForward=0;
            }
            if (driveRightSide>=-0.1&&driveRightSide<=0.1){
                driveRightSide=0;
            }
            if (turnRight>=-0.1&&turnRight<=0.1){
                turnRight=0;
            }

            if(gamepad2.dpad_down==false&&gamepad2.dpad_left==false&&gamepad2.dpad_right==false&&gamepad2.dpad_up==false)
            {
                left1Power = Range.clip((-driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
                right1Power = Range.clip((driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;
                left2Power = Range.clip((driveRightSide + driveForward+turnRight)*allPower, -1.0, 1.0) ;
                right2Power = Range.clip((-driveRightSide + driveForward-turnRight)*allPower, -1.0, 1.0) ;
            }
            else
            {
                if(gamepad2.dpad_up){//up button actually works for down function
                    left1Power=-1*speedForMove*allPower;
                    left2Power=-1*speedForMove*allPower;
                    right1Power=-1*speedForMove*allPower;
                    right2Power=-1*speedForMove*allPower;
                }
                else if(gamepad2.dpad_down){//down button actually works for up function
                    left1Power=1*speedForMove*allPower;
                    left2Power=1*speedForMove*allPower;
                    right1Power=1*speedForMove*allPower;
                    right2Power=1*speedForMove*allPower;
                }
                else if(gamepad2.dpad_right){
                    left1Power=-1*speedForSide*allPower;
                    left2Power=1*speedForSide*allPower;
                    right1Power=1*speedForSide*allPower;
                    right2Power=-1*speedForSide*allPower;
                }
                else if(gamepad2.dpad_left){
                    left1Power=1*speedForSide*allPower;
                    left2Power=-1*speedForSide*allPower;
                    right1Power=-1*speedForSide*allPower;
                    right2Power=1*speedForSide*allPower;
                }
            }

            armUp=(gamepad1.left_stick_y)*armPower;
            if (gamepad1.dpad_up)
            {
                armExtend=extendPower;
            }
            else{
                if (gamepad1.dpad_down)
                {
                    armExtend=-extendPower;
                }
                else
                {
                    armExtend=0;
                }
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
            if (robot.armMotorL.getCurrentPosition()> (armMaxPosition+armPositionInitial) && armUp>=0 && gamepad1.left_stick_button==false)
            {
                armUp=0;
            }
            if(robot.armMotorL.getCurrentPosition()< (armMinPosition+armPositionInitial) && armUp<=0 && gamepad1.left_stick_button==false)
            {
                armUp=0;
            }
            robot.armMotorL.setPower(armUp);
            robot.armMotorR.setPower(armUp);


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
            if(gamepad1.right_bumper)
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
                if (gamepad1.left_bumper)
                {
                    robot.IO_Motor.setPower(-intakePower);
                }
                else
                {
                    robot.IO_Motor.setPower(0);
                }
            }
            if (gamepad1.y)
            {
                robot.IO_Servo_Left.setPosition(IOLeftServoOpen);
                robot.IO_Servo_Right.setPosition(IORightServoOpen);
            }
            if (gamepad1.x)
            {
                robot.IO_Servo_Right.setPosition(IORightServoOpen);
                robot.IO_Servo_Left.setPosition(IOLeftServoHalfOpen);
            }
            if (gamepad1.b)
            {
                robot.IO_Servo_Right.setPosition(IORightServoHalfOpen);
                robot.IO_Servo_Left.setPosition(IOLeftServoOpen);
            }
            if (gamepad1.a)
            {
                armPositionInitial=robot.armMotorL.getCurrentPosition();
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
            telemetry.addData("ArmMaximum",armMaxPosition);
            telemetry.addData("ArmInitial",armPositionInitial);
            telemetry.addData("ArmPosition%7d", robot.armMotorL.getCurrentPosition());
            telemetry.addData("ArmPosition2" , robot.armMotorR.getCurrentPosition());
            telemetry.addData("a button",gamepad1.a);

            telemetry.update();

        }

    }
}
