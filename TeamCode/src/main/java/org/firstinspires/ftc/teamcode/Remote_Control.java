package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class Remote_Control extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

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

            //buttons and gampad on remote
            float yAxis = -gamepad1.left_stick_y;
            float xAxis = gamepad1.left_stick_x;
            boolean upArrow = gamepad1.dpad_up;
            boolean downArrow = gamepad1.dpad_down;
            boolean leftArrow = gamepad1.dpad_left;
            boolean rightArrow = gamepad1.dpad_right;
            boolean armUp = gamepad1.a;
            boolean armDown = gamepad1.x;
            boolean clawClose = gamepad1.b;
            boolean clawOpen = gamepad1.y;

            //double angle = yAxis/xAxis;
            //double medSpeed = 0.2679;
            //double lowSpeed = 0.0875;
            double maxSpeed;

            //determining the power based on degree on angle on joystick
            //if (medSpeed <= angle & angle <= -medSpeed){
            //    maxSpeed = 0.8;
            //}
            //else if (lowSpeed <= angle & angle <= -lowSpeed){
            //    maxSpeed = 0.6;
            //}
            //else{
            //    maxSpeed = 0.4;
            //}';
            maxSpeed = 1;

            leftPower   = Range.clip(yAxis + xAxis, -maxSpeed, maxSpeed);
            rightPower  = Range.clip(yAxis - xAxis, -maxSpeed, maxSpeed);

            //joysticks
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();


            //buttons
            //makeing the robot face the cardinal directions of the board when buttons pressed
            if (upArrow == true) {
                robot.gyroTurn(0.6, 0);
            }
            if (downArrow == true){
                robot.gyroTurn(0.6, 90);
            }
            if (leftArrow == true){
                robot.gyroTurn(0.6, 270);
            }
            if (rightArrow == true){
                robot.gyroTurn(0.6, 360);
            }
            if (armUp == true){
                robot.liftArm.setPower(0.6);
            }
            if (armDown == true){
                robot.liftArm.setPower(-0.6);
            }
            if (clawOpen == true){
                robot.claw.setPosition(0.6);
            }
            if (clawClose == true){
                robot.claw.setPosition(-0.6);
            }
        }
    }
}
