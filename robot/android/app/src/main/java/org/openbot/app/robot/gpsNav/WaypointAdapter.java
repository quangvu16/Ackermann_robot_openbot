package org.openbot.app.robot.gpsNav;

import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageButton;
import android.widget.TextView;
import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.openbot.app.robot.R;

/**
 * Mission waypoint list, one row per waypoint: index, coordinates and the live distance to the
 * robot. Mirrors the waypoint table in Mission Planner's Flight Plan screen.
 */
public class WaypointAdapter extends RecyclerView.Adapter<WaypointAdapter.ViewHolder> {

  public interface OnWaypointActionListener {
    void onWaypointDelete(int position);

    void onWaypointSelect(int position);
  }

  private final List<Waypoint> items = new ArrayList<>();
  private final OnWaypointActionListener listener;
  private int activePosition = -1;
  private double robotLatitude = Double.NaN;
  private double robotLongitude = Double.NaN;

  public WaypointAdapter(OnWaypointActionListener listener) {
    this.listener = listener;
  }

  public void setWaypoints(List<Waypoint> waypoints) {
    items.clear();
    if (waypoints != null) {
      items.addAll(waypoints);
    }
    notifyDataSetChanged();
  }

  public void setActivePosition(int position) {
    int previous = activePosition;
    activePosition = position;
    if (previous >= 0 && previous < items.size()) {
      notifyItemChanged(previous);
    }
    if (activePosition >= 0 && activePosition < items.size()) {
      notifyItemChanged(activePosition);
    }
  }

  public void setRobotPosition(double latitude, double longitude) {
    robotLatitude = latitude;
    robotLongitude = longitude;
    notifyDataSetChanged();
  }

  @Override
  @NonNull
  public ViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
    View view =
        LayoutInflater.from(parent.getContext()).inflate(R.layout.item_waypoint, parent, false);
    return new ViewHolder(view);
  }

  @Override
  public void onBindViewHolder(@NonNull ViewHolder holder, int position) {
    Waypoint waypoint = items.get(position);
    holder.index.setText(String.format(Locale.US, "%d", position + 1));

    if (!waypoint.getName().isEmpty()) {
      holder.name.setText(waypoint.getName());
      holder.name.setVisibility(View.VISIBLE);
    } else {
      holder.name.setVisibility(View.GONE);
    }

    holder.coords.setText(
        String.format(Locale.US, "%.7f, %.7f", waypoint.getLatitude(), waypoint.getLongitude()));

    if (!Double.isNaN(robotLatitude) && !Double.isNaN(robotLongitude)) {
      holder.distance.setText(
          String.format(Locale.US, "%.0f m", waypoint.distanceFrom(robotLatitude, robotLongitude)));
      holder.distance.setVisibility(View.VISIBLE);
    } else {
      holder.distance.setVisibility(View.GONE);
    }

    boolean active = position == activePosition;
    holder.itemView.setAlpha(active ? 1f : 0.75f);
    holder.delete.setOnClickListener(
        v -> listener.onWaypointDelete(holder.getBindingAdapterPosition()));
    holder.itemView.setOnClickListener(
        v -> listener.onWaypointSelect(holder.getBindingAdapterPosition()));
  }

  @Override
  public int getItemCount() {
    return items.size();
  }

  static class ViewHolder extends RecyclerView.ViewHolder {
    final TextView index;
    final TextView name;
    final TextView coords;
    final TextView distance;
    final ImageButton delete;

    ViewHolder(@NonNull View itemView) {
      super(itemView);
      index = itemView.findViewById(R.id.waypoint_index);
      name = itemView.findViewById(R.id.waypoint_name);
      coords = itemView.findViewById(R.id.waypoint_coords);
      distance = itemView.findViewById(R.id.waypoint_distance);
      delete = itemView.findViewById(R.id.waypoint_delete);
    }
  }
}
