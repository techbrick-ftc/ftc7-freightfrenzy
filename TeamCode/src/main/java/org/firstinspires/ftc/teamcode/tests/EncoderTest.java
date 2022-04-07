// Used in troubleshooting the encoders on the arm

package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Disabled
@TeleOp(name="EncoderTest", group="test")
public class EncoderTest extends AutoImport{
    public EncoderTest() { super(31, -56, 225, 150, 255, 150); }

    FieldCentric drive = new FieldCentric();

    public boolean driverAbort() {
        return gamepad1.y;
    }

    private int height = 0;
    @Override
    public void runOpMode() {
        super.runOpMode();
        
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();
        
        waitForStart();
        
        while (opModeIsActive()) {

            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            if (cur2.dpad_up && !prev2.dpad_up && (height < 3)) {
                height++;
                setArm(armY, armYEnc[height], 0.5);
            } else if (cur2.dpad_down && !prev2.dpad_down && (height > 0)) {
                height--;
                setArm(armY, armYEnc[height], 0.5);
            }

            packet.put("ArmX", armX.getCurrentPosition());
            packet.put("ArmY", armX.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }
        }
    }
}