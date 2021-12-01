package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.external.samples.*;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")
public class Remote_Control extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

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

            double angle = yAxis/xAxis;
            double medSpeed = 0.2679;
            double lowSpeed = 0.0875;
            double maxSpeed;

            //determining the power based on degree on angle on joystick
            if (medSpeed <= angle & angle <= -medSpeed){
                maxSpeed = 0.6;
            }
            else if (lowSpeed <= angle & angle <= -lowSpeed){
                maxSpeed = 0.4;
            }
            else{
                maxSpeed = 0;
            }

            leftPower   = Range.clip(yAxis + xAxis, -maxSpeed, maxSpeed);
            rightPower  = Range.clip(yAxis - xAxis, -maxSpeed, maxSpeed);

            //makeing the robot face the cardinal directions of the board when buttons pressed
            if (upArrow == true){
                gyroTurn(0.6, 0);
            }
            if (downArrow == true){
                gyroTurn(0.6, 90);
            }
            if (leftArrow == true){
                gyroTurn(0.6, 270);
            }
            if (rightArrow == true){
                gyroTurn(0.6, 360);
            }
        }
    }
}
