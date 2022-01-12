package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="ArmTest", group="test")
public class ArmTest extends AutoImport {

    public ArmTest() { super(0, 0, 0, 0, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            driveUsingIMU2(-90, 0.3, armX, AxesOrder.ZYX, getImu2());
            setArm(0, 0.5);

            while (opModeIsActive()) {
                packet.put("armXAngle", getImu2().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                packet.put("armYAngle", getImu2().getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
                packet.put("robotAngle", getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                packet.put("armXToRobot", wrap(getImu2().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}