package frc.robot.util;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

/**
 * Minimal adapter that lets a navX (AHRS) be used like a CTRE Pigeon2 for the
 * common calls
 * used in most codebases: getYaw().getValueAsDouble() and setYaw(...).
 *
 * Notes:
 * - getYaw() returns a "status signal" with getValueAsDouble() for drop-in call
 * sites.
 * - setYaw(deg) zeros yaw then applies an angle adjustment so reported yaw ==
 * deg.
 * - Angles are in degrees (CCW+), matching typical WPILib Rotation2d usage.
 */
public class NavXPigeon2 {
  /** Simple stand-in for Phoenix 6 StatusSignal<Double>. */
  public static final class StatusSignalDouble {
    private final DoubleSupplier supplier;

    public StatusSignalDouble(DoubleSupplier supplier) {
      this.supplier = supplier;
    }

    /** Mirrors Phoenix’s API commonly used in codebases. */
    public double getValueAsDouble() {
      return supplier.getAsDouble();
    }

    /** No-op for compatibility. */
    public StatusSignalDouble refresh() {
      return this;
    }
  }

  private final AHRS navx;
  private final StatusSignalDouble yaw;
  private final StatusSignalDouble pitch;
  private final StatusSignalDouble roll;

  /** Construct on MXP (SPI) like most FRC bots. */
  public NavXPigeon2() {
    this(SPI.Port.kMXP);
  }

  public NavXPigeon2(SPI.Port port) {
    this(new AHRS(port));
  }

  public NavXPigeon2(AHRS ahrs) {
    this.navx = ahrs;
    yaw = new StatusSignalDouble(() -> navx.getAngle());
    pitch = new StatusSignalDouble(() -> navx.getPitch());
    roll = new StatusSignalDouble(() -> navx.getRoll());
  }

  /** Pigeon2-like accessors. */
  public StatusSignalDouble getYaw() {
    return yaw.refresh();
  }

  public StatusSignalDouble getPitch() {
    return pitch.refresh();
  }

  public StatusSignalDouble getRoll() {
    return roll.refresh();
  }

  /** Set reported yaw to the given degrees (zero then offset). */
  public void setYaw(double degrees) {
    navx.zeroYaw(); // make current reading 0
    navx.setAngleAdjustment(degrees); // shift to requested heading
  }

  public void setYaw(Rotation2d rot) {
    setYaw(rot.getDegrees());
  }

  /** Convenience helpers. */
  public void reset() {
    navx.reset();
  } // resets displacement, etc.

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw().getValueAsDouble());
  }

  public AHRS getAhrs() {
    return navx;
  } // optional escape hatch
}
