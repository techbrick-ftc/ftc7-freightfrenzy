package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="RedAutoWarehouseDeep", group="redAuto")
public class RedAutoWarehouseDeep extends AutoImport {

    public RedAutoWarehouseDeep() { super(65, 5, 68, 215, 158, 215); } // change

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            sleep(1000);
            slauto.drive(47, -14, 0, 0.75, this);
            deposit(true);
            sleep(500);
            shimmy(1, 1, 200);
            deposit(false);

            // Goes into the warehouse
            slauto.drive(60, 0, 0, 0.5, 0, this, false, true);
            slauto.drive(67, 15, 0, 0.75, 3000, this, false, false);
            slauto.drive(65, 30, 0, 0.75, 3000, this, false, false);
            slauto.drive(40, 30, 0, 0.75, this);

            // Lowers arm
            setArm(0, 1);
            while (armY.isBusy()){
                sleep(10);
            }

            stopCamera();
        }
    }
}
