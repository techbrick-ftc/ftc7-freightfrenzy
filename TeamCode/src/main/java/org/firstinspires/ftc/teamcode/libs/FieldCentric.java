// This is the class that allows us to use field centric in teleop.

package org.firstinspires.ftc.teamcode.libs;
import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

public class FieldCentric {
    // Variable setup, all will be explained within code
    private DcMotor[] motors; // should be fl, rl, fr, rr
    private double currentAngle;
    private double offsetAngle = 0;
    public double rotatedX;
    public double rotatedY;

    public void setUp(DcMotor[] motors) {
        this.motors = motors;
        newOffset();
    }

    private void getAngle() {
        currentAngle = wrap((getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offsetAngle) + Math.PI);
    }

    public void Drive(double x, double y, double turn) {

        getAngle();

        rotatedX = x * Math.cos(-currentAngle) - y * Math.sin(-currentAngle);
        rotatedY = y * Math.cos(currentAngle) - x * Math.sin(currentAngle);

        // Powers set here are swapped around because the front of the robot should be the side
        motors[0].setPower(rotatedY - rotatedX + turn); //fl, effectively fr
        motors[1].setPower(rotatedY + rotatedX - turn); //rl, effectively fl
        motors[2].setPower(rotatedY + rotatedX + turn); //fr, effectively rr
        motors[3].setPower(rotatedY - rotatedX - turn); //rr, effectively rl
    }

    public void newOffset() {
        offsetAngle = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - Math.PI/2;
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }
}
