package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static frc.robot.subsystems.ArmSubsystem.constrainSetpoint;

import java.util.ArrayList;

class AngleConstrainingTest {
    ArrayList<Pair<Rotation2d, Rotation2d>> constraints = new ArrayList<>();

    @BeforeEach
    void emptyConstraintsList() {
        constraints.clear();
    }

    @Test
    void constrainSetpointInOneLimit() {
        constraints.add(Pair.of(Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(2.0)));
        assertEquals(1.0,
                constrainSetpoint(1.0, constraints));
    }

    @Test
    void constrainSetpointInTwoLimits() {
        constraints.add(Pair.of(Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(1.0)));
        constraints.add(Pair.of(Rotation2d.fromRadians(2.0), Rotation2d.fromRadians(3.0)));
        assertEquals(0.9,
                constrainSetpoint(0.9, constraints));
    }

    @Test
    void shiftingSetpointDownIntoOneLimit() {
        constraints.add(Pair.of(Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(2.0)));
        assertEquals(2.0,
                constrainSetpoint(2.7, constraints));
    }

    @Test
    void shiftingSetpointUpIntoOneLimit() {
        constraints.add(Pair.of(Rotation2d.fromRadians(1.0), Rotation2d.fromRadians(2.0)));
        assertEquals(1.0,
                constrainSetpoint(0.7, constraints));
    }

    @Test
    void shiftingSetpointUpIntoTwoLimits() {
        constraints.add(Pair.of(Rotation2d.fromRadians(1.0), Rotation2d.fromRadians(1.5)));
        constraints.add(Pair.of(Rotation2d.fromRadians(2.0), Rotation2d.fromRadians(3.0)));
        assertEquals(1.0,
                constrainSetpoint(0.7, constraints));
    }

    @Test
    void shiftingSetpointDownIntoTwoLimits() {
        constraints.add(Pair.of(Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(0.5)));
        constraints.add(Pair.of(Rotation2d.fromRadians(1.0), Rotation2d.fromRadians(1.5)));
        assertEquals(1.5,
                constrainSetpoint(1.7, constraints));
    }

    @Test
    void shiftingSetpointDownBetweenTwoLimits() {
        constraints.add(Pair.of(Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(0.5)));
        constraints.add(Pair.of(Rotation2d.fromRadians(1.5), Rotation2d.fromRadians(3.0)));
        assertEquals(0.5,
                constrainSetpoint(0.9, constraints));
    }

    @Test
    void shiftingSetpointUpBetweenTwoLimits() {
        constraints.add(Pair.of(Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(0.5)));
        constraints.add(Pair.of(Rotation2d.fromRadians(1.5), Rotation2d.fromRadians(3.0)));
        assertEquals(1.5,
                constrainSetpoint(1.1, constraints));
    }


    @Test
    void shiftingSetpointExactlyBetweenTwoLimits() {
        constraints.add(Pair.of(Rotation2d.fromRadians(0.0), Rotation2d.fromRadians(0.5)));
        constraints.add(Pair.of(Rotation2d.fromRadians(1.5), Rotation2d.fromRadians(3.0)));
        assertEquals(0.5,
                constrainSetpoint(1.0, constraints));
    }
}
