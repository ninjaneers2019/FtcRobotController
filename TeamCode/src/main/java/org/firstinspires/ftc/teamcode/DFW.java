package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class DFW extends LinearOpMode{
    Ninjabot robot;
    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        int FORWARD = 1;
        int BACKWARD = 3;
        int ROTATE_LEFT = 5;
        int ROTATE_RIGHT = 6;

        Ninjabot robot;
        robot = new Ninjabot(hardwareMap, this);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
// zero out the motors counters

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (!opModeIsActive());

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);

//set power for all wheels indefinitely
        //Put moves here

        robot.driveTo(robot.convert(10), FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        //robot.driveTo(950, ROTATE_LEFT);
        //while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        //robot.driveTo(3000, BACKWARD);
        //while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        //robot.driveTo(robot.convert(2), ROTATE_RIGHT);
    }

    }
