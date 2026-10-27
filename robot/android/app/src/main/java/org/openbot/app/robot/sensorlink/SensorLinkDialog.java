package org.openbot.app.robot.sensorlink;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.CheckBox;
import android.widget.RadioGroup;
import androidx.appcompat.app.AlertDialog;
import com.google.android.material.textfield.TextInputEditText;
import com.google.android.material.textfield.TextInputLayout;
import org.openbot.app.robot.R;

/** Off / server / client, the port to serve on, and the other phone's address. */
public final class SensorLinkDialog {

  private SensorLinkDialog() {}

  public static void show(Context context) {
    SensorLinkSettings s = SensorLinkSettings.load(context);
    View view = LayoutInflater.from(context).inflate(R.layout.dialog_sensor_link, null);
    RadioGroup mode = view.findViewById(R.id.sensor_link_mode);
    TextInputLayout portLayout = view.findViewById(R.id.sensor_link_port_layout);
    TextInputEditText port = view.findViewById(R.id.sensor_link_port);
    TextInputLayout addressLayout = view.findViewById(R.id.sensor_link_address_layout);
    TextInputEditText address = view.findViewById(R.id.sensor_link_address);
    CheckBox useGps = view.findViewById(R.id.sensor_link_use_gps);

    mode.check(
        s.mode == SensorLinkSettings.Mode.SERVER
            ? R.id.sensor_link_mode_server
            : s.mode == SensorLinkSettings.Mode.CLIENT
                ? R.id.sensor_link_mode_client
                : R.id.sensor_link_mode_off);
    port.setText(String.valueOf(s.port));
    address.setText(s.address);
    useGps.setChecked(s.useRemoteGps);

    // Show only the fields the chosen mode uses.
    Runnable showFields =
        () -> {
          int id = mode.getCheckedRadioButtonId();
          portLayout.setVisibility(id == R.id.sensor_link_mode_server ? View.VISIBLE : View.GONE);
          boolean client = id == R.id.sensor_link_mode_client;
          addressLayout.setVisibility(client ? View.VISIBLE : View.GONE);
          useGps.setVisibility(client ? View.VISIBLE : View.GONE);
        };
    mode.setOnCheckedChangeListener((g, id) -> showFields.run());
    showFields.run();

    AlertDialog dialog =
        new AlertDialog.Builder(context)
            .setTitle(R.string.sensor_link_title)
            .setView(view)
            .setPositiveButton(R.string.sensor_link_save, null)
            .setNegativeButton(R.string.sensor_link_cancel, null)
            .create();
    dialog.setOnShowListener(
        d ->
            dialog
                .getButton(AlertDialog.BUTTON_POSITIVE)
                .setOnClickListener(
                    v -> {
                      int id = mode.getCheckedRadioButtonId();
                      SensorLinkSettings.Mode m =
                          id == R.id.sensor_link_mode_server
                              ? SensorLinkSettings.Mode.SERVER
                              : id == R.id.sensor_link_mode_client
                                  ? SensorLinkSettings.Mode.CLIENT
                                  : SensorLinkSettings.Mode.OFF;
                      int p;
                      try {
                        p = Integer.parseInt(String.valueOf(port.getText()).trim());
                      } catch (NumberFormatException e) {
                        p = -1;
                      }
                      if (m == SensorLinkSettings.Mode.SERVER && !SensorLinkSettings.isValidPort(p)) {
                        portLayout.setError(context.getString(R.string.sensor_link_bad_port));
                        return;
                      }
                      String a = String.valueOf(address.getText()).trim();
                      if (m == SensorLinkSettings.Mode.CLIENT && SensorLinkProtocol.baseUrl(a) == null) {
                        addressLayout.setError(context.getString(R.string.sensor_link_client_no_address));
                        return;
                      }
                      s.mode = m;
                      if (SensorLinkSettings.isValidPort(p)) s.port = p;
                      s.address = a;
                      s.useRemoteGps = useGps.isChecked();
                      s.save(context);
                      SensorLink.get().restart(context);
                      dialog.dismiss();
                    }));
    dialog.show();
  }
}
