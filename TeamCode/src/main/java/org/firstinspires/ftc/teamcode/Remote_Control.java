package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class Remote_Control extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        Ninjabot robot;
        robot = new Ninjabot(hardwareMap, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        ModernRoboticsI2cGyro gyro = null;

        while (opModeIsActive()){
            double leftPower;
            double rightPower;

            //buttons and game pad on remote
            float yAxis = -gamepad1.left_stick_y;
            float xAxis = gamepad1.left_stick_x;
            boolean upArrow = gamepad1.dpad_up;
            boolean downArrow = gamepad1.dpad_down;
            boolean leftArrow = gamepad1.dpad_left;
            boolean rightArrow = gamepad1.dpad_right;
            boolean armUp = gamepad1.y;//this doing anything
            boolean armDown = gamepad1.a;//
            boolean clawClose = gamepad1.x;
            boolean clawOpen = gamepad1.b;

            //double angle = yAxis/xAxis; double medSpeed = 0.2679; double lowSpeed = 0.0875;
            double maxSpeed = 0.6;
            boolean spinPower = false;
            double speedspin = 0;
            spinPower = gamepad1.left_bumper;
            if(spinPower == true){
                speedspin = 1;
            }
            else if(gamepad1.right_bumper == true){
                speedspin = -1;
            }
            else{
                speedspin = 0;
            }
            robot.spinner.setPower(speedspin);



            double upPower = Range.clip( gamepad1.left_trigger , -maxSpeed, maxSpeed);//might need an increase
            double downPower = Range.clip( gamepad1.right_trigger, -maxSpeed, maxSpeed);
            if(upPower > 0){

                robot.liftArm.setPower(upPower);
            } else if(downPower > 0){
                robot.liftArm.setPower(-downPower);
            }else{
                robot.liftArm.setPower(0);
            }
            double boost = 0;
            //robot.liftArm.setPower();///////////////////////
            if (armDown == true){
                //  robot.liftArm.setPower(0.6);//0.6
                boost = 0.4;
            }
            if(armUp == true){
                boost = -0.4;
            }


            leftPower   = Range.clip(yAxis + xAxis, -maxSpeed-boost, maxSpeed+boost);
            rightPower  = Range.clip(yAxis - xAxis, -maxSpeed-boost, maxSpeed+boost);

            //joysticks
          //  robot.leftDrive.setPower(leftPower);
          //  robot.rightDrive.setPower(rightPower);

            robot.leftDrive.setPower(rightPower);
            robot.rightDrive.setPower(leftPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();


            //buttons
            //making the robot face the cardinal directions of the board when buttons pressed
            if (upArrow == true) {
                robot.gyroTurn(0.8, 0);
            }
            if (downArrow ==true){
                robot.gyroTurn(0.8, 90);
            }
            if (leftArrow == true){
                robot.gyroTurn(0.6, 270);
            }
            if (rightArrow == true){
                robot.gyroTurn(0.6, 360);
            }
            //if (armUp == true){
            //    robot.liftArm.setPower(-0.7);//-0.6
            //}


            if (clawOpen == true){
                robot.claw.setPosition(0.8);
            }
            if (clawClose == true){
                robot.claw.setPosition(0.1);//this claw position was -0.2
            }


            sleep(10);
        }
    }
}
