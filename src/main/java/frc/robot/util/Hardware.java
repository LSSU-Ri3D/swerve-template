package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.function.Supplier;

// Applies a motor/sensor config with a few retries. Configs fail now and then at boot when the CAN
// bus is busy, and a motor that silently missed its config (no current limit, wrong inversion) is
// way worse than one that yells about it, so if every try fails it puts an error on the dashboard.
public final class Hardware {
  private Hardware() {}

  private static final int ATTEMPTS = 5;

  public static void configure(String device, Supplier<StatusCode> apply) {
    StatusCode result = StatusCode.OK;
    for (int i = 0; i < ATTEMPTS; i++) {
      result = apply.get();
      if (result.isOK()) {
        return;
      }
    }
    fail(device, result.getName());
  }

  public static void configureRev(String device, Supplier<REVLibError> apply) {
    REVLibError result = REVLibError.kOk;
    for (int i = 0; i < ATTEMPTS; i++) {
      result = apply.get();
      if (result == REVLibError.kOk) {
        return;
      }
    }
    fail(device, result.name());
  }

  // the Alert never gets closed on purpose, it has to stay up for the rest of the match
  @SuppressWarnings("resource")
  private static void fail(String device, String reason) {
    new Alert(device + " didn't take its config: " + reason, AlertType.kError).set(true);
  }
}
