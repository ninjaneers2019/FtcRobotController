package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class FDP extends LinearOpMode{
    Ninjabot robot;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        int FORWARD = 1;
        int BACKWARD = 3;
        int ROTATE_LEFT = 5;
        int ROTATE_RIGHT = 6;
        int TANK_LEFT= 7;
        int TANK_RIGHT= 8;

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
        robot.driveTo(robot.convert(25 ), FORWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.liftArm.setTargetPosition(70); // move arm down to drop item in claw 80 is drop to floor
        robot.liftArm.setPower(1);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);

        robot.claw.setPosition(0.8); //claw open, position 0 is close
        sleep(1000);
        telemetry.addData("Status", "Open");
        telemetry.update();

        robot.liftArm.setTargetPosition(0); // move arm down to drop item in claw 80 is drop to floor
        robot.liftArm.setPower(1);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.driveTo(robot.convert(25), BACKWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.driveTo(500, ROTATE_LEFT); //950 is equal to a 180 degree turn of the robot in rotate turns
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.driveTo(robot.convert(45), FORWARD); //drive backward towards the duck carosel
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.spinner.setPower(0.5); //spinner for the duck
        sleep(2000);
        robot.spinner.setPower(0);// turn off spinner

        robot.driveTo(robot.convert(5), BACKWARD);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

        robot.driveTo(500, ROTATE_RIGHT);
        while (!robot.targetReached() && opModeIsActive()) robot.updateWheelTelemetry();

    }
}