% This script can be used to check delta robot calubration algorithm.
% We assume carriges moving along vertical towers
format shortg;
automatic_calibration = true;
adjustments = {"configured_endstop_offsets = ";
  "configured_rod_length = ";
  "configured_tower_radius = ";
  "configured_toolhead_home_z = ";
  "configured_tower_angles = "
  };


function forward = forward_delta(towers, rods)
  p12 = towers(2,:) - towers(1,:);
  d = sqrt(sum(p12.^2));
  ex = p12 ./ d;
  p13 = towers(3,:) - towers(1,:);
  i = dot(ex, p13);
  iex = ex * i;
  ey = p13 - iex;
  j = sqrt(sum(ey.^2));
  ey = ey ./ j;
  ez = cross(ex, ey);
  Xnew = (rods(1) ^2 - rods(2) ^2 + d ^2) ./ (2 * d);

  Ynew = (rods(1) ^2 - rods(3) ^2 + i ^2 + j ^2 - 2 * i * Xnew) / (2 * j);

  Znew = sqrt(rods(1) ^2 - Xnew ^2 - Ynew ^2);

  forward = towers(1,:) + ex .* Xnew;
  forward += ey .* Ynew;
  forward += ez .* -Znew;

endfunction

function towers_z = reverse_delta(toolhead, towers_xy, rods)
  % This assumes perfectly vertical towers
  towers_z = toolhead(3) + sqrt(rods .^2 - (toolhead(1) - towers_xy(:, 1)).^2 - (toolhead(2) - towers_xy(:, 2)).^2);
endfunction

%##########################################################################################
%           Geometry of actual kinematics in absolute coordinates
%##########################################################################################
%                           angle, radius, height above absolute zero
actual_endstop_positions = [deg2rad(210), 120, 400   % A tower
                          deg2rad(330), 120, 400   % B tower
                          deg2rad(90), 120, 400];    % C tower
                        % azimuth, elevation, scale
[x, y, z] = sph2cart([deg2rad(0), deg2rad(90), 1
                      deg2rad(0), deg2rad(90), 1
                      deg2rad(0), deg2rad(90), 1]);

actual_tower_direction = [x, y, z];
actual_rod_length = [250;
                    250;
                    250];

reachable_radius = 100;

                    % X, Y, Z
actual_plane_slope = [0.0, 0.01, 0];

[x, y, z] = pol2cart(actual_endstop_positions);
actual_tower_top_position = [x, y, z]

actual_tool_position_top = forward_delta(actual_tower_top_position, actual_rod_length);

%##########################################################################################
%           Configured values of kinematics in software
%##########################################################################################
configured_toolhead_home_z = actual_tool_position_top(3) - 0.25
configured_endstop_offsets = [0; 0; 0]

configured_tower_radius = 120
configured_tower_angles = [210; 330; 90]
configured_rod_length = ones(3, 1) .* 250

plot_title = "Before leveling";
for k = 1:9
  [x, y] = pol2cart(deg2rad(configured_tower_angles), ones(3, 1) .* configured_tower_radius);

  % these values will correspond to position of actual_tower_top_position - configured_endstop_offsets
  virtual_tower_top_positions = [x, y, reverse_delta([0, 0, configured_toolhead_home_z], [x, y], configured_rod_length)];

  % Recalculate top position, including endstop offsets
  actual_tower_top_position = actual_tower_top_position - actual_tower_direction .* configured_endstop_offsets;
  actual_tool_position_top = forward_delta(actual_tower_top_position, actual_rod_length);

  % Measure surface pane
  [x, y, z] = pol2cart(linspace(0, 2*pi, 7)' + 0.5 * pi, ones(7, 1) .* reachable_radius, 2);
  x(end) = 0;
  y(end) = 0;

  measurements = [x, y, z]; % In virtual coordinates
  for i = 1:7
    measurement_error = 9999;
    while (abs(measurement_error) > 0.001)
      virt_cariges = reverse_delta(measurements(i, :), virtual_tower_top_positions(:, 1:2), configured_rod_length);
      act_pos = forward_delta(actual_tower_top_position - actual_tower_direction .* (virtual_tower_top_positions - virt_cariges), actual_rod_length);
      measurement_error = (act_pos(1) * actual_plane_slope(1) + act_pos(2) * actual_plane_slope(2) + actual_plane_slope(3)) - act_pos(3);
      measurements(i, 3) += measurement_error;
    endwhile
  endfor

  % Should be positive, where plane is above virtual Z=0 and negaive, where below
  measurements

  % Plot Z=0 plane relative to actual base plane
  [x, y] = meshgrid(linspace(-configured_tower_radius, configured_tower_radius, 100));
  z = zeros(size(x));
  for i = 1:100
    for j = 1:100
      if (sqrt(x(i, j)^2 + y(i, j)^2) > reachable_radius)
        continue;
      endif
      virt_cariges = reverse_delta([x(i, j), y(i, j), 0], virtual_tower_top_positions(:, 1:2), configured_rod_length);
      act_pos = forward_delta(actual_tower_top_position - actual_tower_direction .* (virtual_tower_top_positions - virt_cariges), actual_rod_length);
      z(i, j) = act_pos(3) - (act_pos(1) * actual_plane_slope(1) + act_pos(2) * actual_plane_slope(2) + actual_plane_slope(3));
    endfor
  endfor

  figure(k);

  mesh(x, y, z);
  hold on;

  [v, i] = max(z);
  [v, j] = max(v);

  [u, a] = min(z);
  [u, b] = min(u);

  points = [x(i(j), j), y(i(j), j), v;
            x(a(b), b), y(a(b), b), u];

  scatter3(points(:, 1), points(:, 2), points(:, 3));

  text(points(:, 1), points(:, 2), points(:, 3) + [0.1; -0.1] * (v - u),
          {sprintf("%f", v), sprintf("%f", u)});

  xlabel("x");
  ylabel("y");
  title(plot_title);
  plot_title = sprintf("After %d cycles", k);
  hold off;

  cmd = strjoin({
    "mono leveling.exe",
    sprintf("%f", configured_rod_length(1)),
    sprintf("%f", configured_tower_radius),
    sprintf("%f", configured_toolhead_home_z),
    sprintf("%f", configured_endstop_offsets(1)),
    sprintf("%f", configured_endstop_offsets(2)),
    sprintf("%f", configured_endstop_offsets(3)),
    sprintf("%f", configured_tower_angles(1)),
    sprintf("%f", configured_tower_angles(2)),
    sprintf("%f", configured_tower_angles(3))
  }, " ");
  for i = 1:7
    cmd = strjoin({cmd,
      sprintf("%f", measurements(i, 1)),
      sprintf("%f", measurements(i, 2)),
      sprintf("%f", measurements(i, 3))
      }, " ");
  endfor
  if (automatic_calibration)
    [_, calibration] = system(cmd);
    eval(calibration)
  else
    for a = 1:5
      fflush(stdout);
      v = input(adjustments{a}, "s")
      eval(strjoin({adjustments{a}, v}, " "))
    endfor
  endif

endfor





