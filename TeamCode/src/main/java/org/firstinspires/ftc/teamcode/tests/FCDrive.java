package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="FCDrive", group="test")
public class FCDrive extends AutoImport {

    public FCDrive() { super(31, -56, 225, 150, 255, 150); }
    FieldCentric drive = new FieldCentric();

    @Override
    public void runOpMode() {
        super.runOpMode();

        drive.setUp(new DcMotor[] {fl, rl, fr, rr});

        while(opModeIsActive()) {
            drive.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.a) {
                drive.newOffset();
            }

            packet.put("x", gamepad1.left_stick_x);
            packet.put("y", -gamepad1.left_stick_y);
            packet.put("turn", gamepad1.right_stick_x);
            packet.put("rx", drive.rotatedX);
            packet.put("ry", drive.rotatedY);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
