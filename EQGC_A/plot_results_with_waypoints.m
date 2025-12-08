% plot_results_with_waypoints.m
% 自动读取 C++ 生成的配置和结果进行绘图
% 包含航迹点和实际飞行轨迹的对比显示
clear; clc; close all;

%% 1. 文件路径设置
base_dir = 'D:\4-program\1\eqgc\result\';
data_file = fullfile(base_dir, 'bao_simulation.csv');
conf_file = fullfile(base_dir, 'mission_config.csv');
waypoint_file = fullfile(base_dir, 'waypoints.csv');

if ~isfile(data_file) || ~isfile(conf_file)
    error('未找到数据文件！请先运行 C++ 程序。');
end

%% 2. 读取数据
% 读取仿真结果
sim_data = readtable(data_file);
% 读取任务配置
mission_conf = readtable(conf_file);

% 读取航迹点（如果存在）
waypoints_exist = isfile(waypoint_file);
if waypoints_exist
    waypoint_data = readtable(waypoint_file);
 wp_lon = waypoint_data.Lon_deg;
  wp_lat = waypoint_data.Lat_deg;
    fprintf('[MATLAB] Loaded %d waypoints\n', length(wp_lon));
else
    warning('未找到航迹点文件: %s', waypoint_file);
  wp_lon = [];
    wp_lat = [];
end

% 提取仿真数据
t = sim_data.Time;
h_km = sim_data.Alt_km;
lon = sim_data.Lon_deg;
lat = sim_data.Lat_deg;
v = sim_data.Vel;
gamma = sim_data.Gamma_deg;
psi = sim_data.Psi_deg;
alpha = sim_data.Alpha_deg;
bank = sim_data.Bank_deg;

% 解析任务配置
% Type: 0=Start, 1=Target, 2=NFZ
start_pt = mission_conf(mission_conf.Type == 0, :);
target_pt = mission_conf(mission_conf.Type == 1, :);
nfz_list = mission_conf(mission_conf.Type == 2, :);

%% 3. 绘制三维轨迹 (Figure 1)
figure(1); set(gcf, 'Color', 'w', 'Position', [100 100 1000 700]);
plot3(lon, lat, h_km, 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'Actual Trajectory'); 
grid on; hold on;
xlabel('Longitude (deg)', 'FontSize', 12); 
ylabel('Latitude (deg)', 'FontSize', 12); 
zlabel('Altitude (km)', 'FontSize', 12);
title('3D Reentry Trajectory with Waypoints', 'FontSize', 14);
view(-30, 45);

% 绘制航迹点（3D视图中用地面投影，高度设为起始高度）
if waypoints_exist && ~isempty(wp_lon)
    % 在3D图中绘制航迹点（投影到初始高度）
    wp_h = ones(size(wp_lon)) * h_km(1); % 所有航迹点设为起始高度
    plot3(wp_lon, wp_lat, wp_h, 'ro-', 'LineWidth', 1.5, 'MarkerSize', 6, ...
  'MarkerFaceColor', 'r', 'DisplayName', 'Planned Waypoints');
    
    % 标注航迹点序号
    for i = 1:length(wp_lon)
        text(wp_lon(i), wp_lat(i), wp_h(i), sprintf(' WP%d', i-1), ...
       'FontSize', 8, 'Color', 'r');
    end
end

% 绘制起点和终点
if ~isempty(start_pt)
    plot3(start_pt.Lon_deg, start_pt.Lat_deg, h_km(1), 'go', 'MarkerSize', 12, ...
          'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start');
end
if ~isempty(target_pt)
    plot3(target_pt.Lon_deg, target_pt.Lat_deg, h_km(end), 'rx', 'MarkerSize', 12, ...
          'MarkerFaceColor', 'r', 'LineWidth', 2, 'DisplayName', 'Target');
end

% 自动绘制所有禁飞区
m2deg = 180 / (pi * 6371000); % 米转度系数估计
for i = 1:height(nfz_list)
    cz_lat = nfz_list.Lat_deg(i);
    cz_lon = nfz_list.Lon_deg(i);
    r_deg = nfz_list.Radius_m(i) * m2deg;
    
    % 绘制圆柱
    [X, Y, Z] = cylinder(r_deg, 30);
    X = X + cz_lon;
    Y = Y + cz_lat;
    Z = Z * 80; % 拉伸高度
    surf(X, Y, Z, 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
         'HandleVisibility', 'off'); % 不显示在图例中
end
legend('Location', 'best', 'FontSize', 10);

%% 4. 绘制地面轨迹 (Figure 2) - 重点对比航迹点和实际轨迹
figure(2); set(gcf, 'Color', 'w', 'Position', [150 150 1000 700]);
hold on; grid on;
xlabel('Longitude (deg)', 'FontSize', 12); 
ylabel('Latitude (deg)', 'FontSize', 12);
title('Ground Track: Planned Waypoints vs Actual Trajectory', 'FontSize', 14);
axis equal;

% 绘制禁飞区圆圈（先画，作为背景）
theta = linspace(0, 2*pi, 50);
for i = 1:height(nfz_list)
    cz_lat = nfz_list.Lat_deg(i);
    cz_lon = nfz_list.Lon_deg(i);
    r_deg = nfz_list.Radius_m(i) * m2deg;
    
xc = r_deg * cos(theta) + cz_lon;
    yc = r_deg * sin(theta) + cz_lat;
    fill(xc, yc, 'r', 'FaceAlpha', 0.25, 'EdgeColor', 'r', 'LineWidth', 1.5, ...
  'DisplayName', sprintf('NFZ %d', i));
end

% 绘制航迹规划路径（折线）
if waypoints_exist && ~isempty(wp_lon)
    plot(wp_lon, wp_lat, 'r--o', 'LineWidth', 2, 'MarkerSize', 8, ...
         'MarkerFaceColor', 'r', 'DisplayName', 'Planned Path');
    
    % 标注航迹点序号
    for i = 1:length(wp_lon)
      text(wp_lon(i), wp_lat(i), sprintf('  %d', i-1), ...
     'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold');
    end
end

% 绘制实际飞行轨迹
plot(lon, lat, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Actual Trajectory');

% 绘制起点和终点
if ~isempty(start_pt)
    plot(start_pt.Lon_deg, start_pt.Lat_deg, 'go', 'MarkerSize', 14, ...
         'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start');
end
if ~isempty(target_pt)
    plot(target_pt.Lon_deg, target_pt.Lat_deg, 'kx', 'MarkerSize', 14, ...
       'LineWidth', 3, 'DisplayName', 'Target');
end

legend('Location', 'best', 'FontSize', 10);

%% 5. 状态绘图 (Figure 3: 飞行状态)
figure(3); set(gcf, 'Color', 'w', 'Position', [200 200 1000 700]);

% 子图1: 高度-速度剖面
subplot(2,2,1); 
plot(v, h_km, 'LineWidth',2, 'Color', 'b'); grid on; set(gca,'XDir','reverse');
xlabel('Velocity (m/s)', 'FontSize', 11); 
ylabel('Altitude (km)', 'FontSize', 11); 
title('H-V Profile', 'FontSize', 12);

% 子图2: 航迹倾角
subplot(2,2,2); 
plot(t, gamma, 'LineWidth',2, 'Color', [0.8 0.4 0]); grid on;
xlabel('Time (s)', 'FontSize', 11); 
ylabel('Gamma (deg)', 'FontSize', 11); 
title('Flight Path Angle', 'FontSize', 12);

% 子图3: 航向角
subplot(2,2,3); 
plot(t, psi, 'LineWidth',2, 'Color', [0.5 0.2 0.8]); grid on;
xlabel('Time (s)', 'FontSize', 11); 
ylabel('Psi (deg)', 'FontSize', 11); 
title('Heading Angle', 'FontSize', 12);

% 子图4: 速度-时间
subplot(2,2,4); 
plot(t, v, 'LineWidth',2, 'Color', [0.2 0.6 0.4]); grid on;
xlabel('Time (s)', 'FontSize', 11); 
ylabel('Velocity (m/s)', 'FontSize', 11); 
title('Velocity vs Time', 'FontSize', 12);

%% 6. 控制量绘图 (Figure 4: 攻角与滚转角)
figure(4); set(gcf, 'Color', 'w', 'Position', [250 250 1000 600]);

% 子图1: 攻角 (Alpha)
subplot(2,1,1); 
plot(t, alpha, 'LineWidth', 2, 'Color', 'b'); 
grid on;
ylabel('Alpha (deg)', 'FontSize', 11); 
title('Angle of Attack', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 11); 

% 子图2: 滚转角/倾侧角 (Bank)
subplot(2,1,2); 
plot(t, bank, 'LineWidth', 2, 'Color', 'r'); 
grid on;
xlabel('Time (s)', 'FontSize', 11); 
ylabel('Bank Angle (deg)', 'FontSize', 11); 
title('Bank Angle', 'FontSize', 12);

%% 7. 显示统计信息
fprintf('\n=== Simulation Statistics ===\n');
fprintf('Total Time: %.2f s\n', t(end));
fprintf('Initial Altitude: %.2f km\n', h_km(1));
fprintf('Final Altitude: %.2f km\n', h_km(end));
fprintf('Initial Velocity: %.2f m/s\n', v(1));
fprintf('Final Velocity: %.2f m/s\n', v(end));
fprintf('Number of Waypoints: %d\n', length(wp_lon));

% 计算最终距离目标的距离
if ~isempty(target_pt)
    final_lon = lon(end);
    final_lat = lat(end);
    target_lon = target_pt.Lon_deg;
    target_lat = target_pt.Lat_deg;
    
    % 简化的距离计算（球面距离）
    dlat = (target_lat - final_lat) * pi/180;
    dlon = (target_lon - final_lon) * pi/180;
    a = sin(dlat/2)^2 + cos(final_lat*pi/180) * cos(target_lat*pi/180) * sin(dlon/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    miss_distance_km = 6371 * c;
    
    fprintf('Miss Distance: %.2f km\n', miss_distance_km);
end

fprintf('===========================\n\n');
