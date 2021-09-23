package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBot {
    private DcMotor flMotor;
    public DcMotor flMotor() { return flMotor; }
    private DcMotor frMotor;
    public DcMotor frMotor() { return frMotor; }
    private DcMotor rlMotor;
    public DcMotor rlMotor() { return rlMotor; }
    private DcMotor rrMotor;
    public DcMotor rrMotor() { return rrMotor; }

    public void setup(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        rlMotor = hardwareMap.get(DcMotor.class, "rl");
        rrMotor = hardwareMap.get(DcMotor.class, "rr");

        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rlMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
