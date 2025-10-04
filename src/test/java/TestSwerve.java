import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class TestSwerve {
    public static void main(String[] args) {
    }

    
    @Override
    public String toString() {
        return "TestSwerve []";
    }


    @Test
    void testInitialization(){
        SwerveMod mod = new SwerveMod();
        assertEquals(1, mod.moduleNumber);        
    }

}