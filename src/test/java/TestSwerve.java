import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;
import frc.robot.Subsystems.SwerveMod;

class TestSwerve {
    public static void main(String[] args) {
    }

    @Test
    void testInitialization(){
        SwerveMod mod = new SwerveMod(1, Constants.Swerve.Mod0.constants);
        assertEquals(1, mod.moduleNumber);        
    }

}