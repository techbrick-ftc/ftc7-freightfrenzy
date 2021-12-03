package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="RedAuto", group="teleop")
public class RedAuto extends AutoImport {

    public RedAuto() { super(65, -40, 75, 110, 200, 110); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to spinner and does spinny
            slauto.drive(55, -60, -90, 0.75, this);
            setSpinny(true, 1000);

            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            slauto.drive(40, -5, 0, 0.5, this);
            runIntake(1, 2000);

            /*
            // Starts lifting arm
            armY.setTargetPosition(-2000);
            armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armY.setPower(1);

            sleep(1000);

            // Goes to the other side of the wall in 3 connected movements
            slauto.drive(50, -40, 0, 0.75, 0, this, false, true);
            slauto.drive(65, 0, 0, 0.5, 0, this, false, false);
            slauto.drive(65, 50, 0, 0.5, 0, this, true, false);
            */

            stopCamera();
        }
    }
}
