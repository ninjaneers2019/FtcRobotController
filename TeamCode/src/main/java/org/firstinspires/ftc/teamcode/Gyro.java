package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Gyro {

    //as given
    public void gyroTurn(double speed, double angle){
        while (opModeIsActive() && !onHeading(speed, angle, 0.1)){
            telemetry.update();
        }
    }
    //as given
    boolean onHeading(double speed, double angle, double PCoeff){
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= 1){
            steer = 0;
            leftSpeed = 0;
            rightSpeed =0;
            onTarget = true;
        }
        else{
            steer       = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        //add info to display on phone
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle){
        double robotError;

        robotError = 1 ; // targetAngle - gyro.getIntergratedZValue();
        while(robotError > 180)         robotError -= 360;
        while(robotError <= 180)        robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff){
        return Range.clip(error * PCoeff, -1, 1);
    }
}
