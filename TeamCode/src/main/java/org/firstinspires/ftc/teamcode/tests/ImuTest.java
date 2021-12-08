package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="ImuTest", group="test")
public class ImuTest extends AutoImport {

    public ImuTest() { super(0, 0, 0, 0, 0, 0); }

    private static BNO055IMU imu2;

    public void runOpMode() {
        super.runOpMode();

        imu2 = hardwareMap.get(BNO055IMU .class, "imuExternal");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu2.initialize(params);

        while (opModeIsActive()) {
            packet.put("x", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
            packet.put("y", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle);
            packet.put("z", imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}