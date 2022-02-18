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
        robot.liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        telemetry.addData("Status", "ClawStatus");
        telemetry.update();
        robot.claw.setPosition(0);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        //Robot will be starting right infront of the docking platform
        robot.driveTo(robot.convert(30), FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.liftArm.setTargetPosition(40); // move arm down to drop item in claw 80 is drop to floor
        robot.liftArm.setPower(1);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.claw.setPosition(0.8); //claw open, position 0 is close
        sleep(500);
        telemetry.addData("Status", "Open");
        telemetry.update();

        robot.liftArm.setTargetPosition(0); // move arm down to drop item in claw 80 is drop to floor
        robot.liftArm.setPower(1);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.driveTo(950, ROTATE_LEFT); //950 is equal to a 180 degree turn of the robot
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.liftArm.setTargetPosition(0); // move arm up to avoid knocking item off
        robot.liftArm.setPower(1);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.claw.setPosition(0); //position 0 is close
        sleep(500);
        telemetry.addData("Status", "close");
        telemetry.update();



        sleep(2000);

        robot.liftArm.setTargetPosition(20);
        robot.liftArm.setPower(0.5);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);

        robot.driveTo(3000, BACKWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.driveTo(robot.convert(2), ROTATE_RIGHT);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();
    }

    }
