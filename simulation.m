%% ENERGY EFFICIENT ACC SIMULATION ENGINE (V46.0)
%  Project: Comparative Analysis of Static vs. Dynamic ACC for EVs
%  Author:  Sidar Angün
%  Date:    January 2026
%  Vehicle: Tesla Model Y Long Range AWD (Calibrated Physics)
%
%  Description:
%  This script generates stochastic traffic scenarios and simulates longitudinal
%  vehicle dynamics to compare energy consumption between a standard PID-based
%  controller and the proposed Kinematic Horizon strategy.
%
%  Usage: Just run the script. It will generate the figures used in the report.
%% -------------------------------------------------------------------------
clear; clc; close all;

%% 1. PHYSICAL PARAMETERS (Tesla Model Y - Blended Braking)
dt = 0.1; 
Target_Distance_m = 500000; % 500 km Target

Mass = 2050; 
Air_Density = 1.225;
Drag_Coeff = 0.23; 
Frontal_Area = 2.5; 
Rolling_Res = 0.012; 
Gravity = 9.81;
Battery_Cap_kWh = 75.0; 
Aux_Load_kW = 0.5;

% REGEN LIMIT
Max_Regen_kW = 75.0; % Battery accepts max 60 kW, excess becomes Heat (Waste).

%% 2. TRAFFIC SCENARIO
Est_N = 300000; 
v_lead = zeros(1, Est_N);
x_lead = zeros(1, Est_N);
x_lead(1) = 500; 

current_v = 110/3.6; target_v = 110/3.6; timer = 0;

fprintf('Generating Traffic Scenario...\n');
for k = 1:Est_N
    timer = timer - dt;
    if timer <= 0
        r = rand();
        if r < 0.30, target_v = (20 + rand*30)/3.6; timer = 30 + rand*30; 
        elseif r < 0.70, target_v = (90 + rand*30)/3.6; timer = 20 + rand*40; 
        else, target_v = (140 + rand*10)/3.6; timer = 15 + rand*20; end 
    end
    diff = target_v - current_v;
    acc = sign(diff) * min(abs(diff), 2.0 * dt); 
    current_v = current_v + acc; if current_v < 0, current_v = 0; end
    v_lead(k) = current_v;
    if k < Est_N, x_lead(k+1) = x_lead(k) + v_lead(k) * dt; end
end

%% 3. SIMULATION
Static = struct('x',0, 'v',110/3.6, 'batt',Battery_Cap_kWh, 'waste',0, 'regen',0, 'finished',false, 'is_dead',false, 'death_km',0, 'death_time',0);
Dynamic = struct('x',0, 'v',110/3.6, 'batt',Battery_Cap_kWh, 'waste',0, 'regen',0, 'finished',false, 'is_dead',false, 'death_km',0, 'death_time',0);

% Analysis Variables
Analysis_Done = false;
Dyn_Arrival_Time = 0;
Dyn_Batt_At_Checkpoint = 0;

% Logging Setup
Log_Ind = 0; Step = 10; Alloc = ceil(Est_N/Step);

L_Dist = zeros(1, Alloc); 
L_Soc_S = zeros(1, Alloc); L_Soc_D = zeros(1, Alloc);
L_Time = zeros(1, Alloc); 
L_Gap_S = zeros(1, Alloc); L_Gap_D = zeros(1, Alloc);
L_Vel_S = zeros(1, Alloc); L_Vel_D = zeros(1, Alloc); 
L_Vel_L = zeros(1, Alloc); 

fprintf('Race Started... Waiting for Static car battery depletion...\n');

k = 1;
while k < Est_N
    
    %% A. STATIC CAR (UPDATED LOGIC: Time Headway)
    if ~Static.finished && ~Static.is_dead
        dist_s = x_lead(k) - Static.x;
        
        % <--- V43 UPDATE: Time Headway Logic --->
        % Instead of fixed 40m, we use 1.5 seconds gap + 10m buffer.
        % This is standard industry practice for ACC.
        safe_gap_s = max(10, Static.v * 1.5); 
        
        accel_s = 1.0 * (dist_s - safe_gap_s) + 3.0 * (v_lead(k) - Static.v);
        
        [Static.v, Static.x, Static.batt, w_s, r_s] = phys_step(Static.v, Static.x, Static.batt, accel_s, dt, Mass, ...
            Air_Density, Drag_Coeff, Frontal_Area, Rolling_Res, Gravity, Aux_Load_kW, Max_Regen_kW);
        
        Static.waste = Static.waste + w_s;
        Static.regen = Static.regen + r_s;
        
        if Static.batt <= 0
            Static.is_dead = true; Static.v = 0; 
            Static.death_km = Static.x/1000; Static.death_time = k*dt;
        end
        if Static.x >= Target_Distance_m, Static.finished = true; end
    end
    
    %% B. DYNAMIC CAR (Kinematic Horizon)
    if ~Dynamic.finished && ~Dynamic.is_dead
        dist_d = x_lead(k) - Dynamic.x;
        rel_v = v_lead(k) - Dynamic.v;
        accel_d = 0;
        
        if rel_v < -0.5
            safe_dist = max(5, dist_d - 20);
            req_decel = (v_lead(k)^2 - Dynamic.v^2) / (2 * safe_dist);
            accel_d = min(-0.1, max(-1.2, req_decel)); 
            if dist_d < 40, accel_d = -3.0; end 
        elseif rel_v > 0.5
            if dist_d > 350, accel_d = 0.4 * ((115/3.6) - Dynamic.v); 
            else, accel_d = 0.6; end
        else
            if dist_d < 100, accel_d = -0.5; else, accel_d = 0; end
        end
        
        [Dynamic.v, Dynamic.x, Dynamic.batt, w_d, r_d] = phys_step(Dynamic.v, Dynamic.x, Dynamic.batt, accel_d, dt, Mass, ...
            Air_Density, Drag_Coeff, Frontal_Area, Rolling_Res, Gravity, Aux_Load_kW, Max_Regen_kW);
        
        Dynamic.waste = Dynamic.waste + w_d;
        Dynamic.regen = Dynamic.regen + r_d;
        
        if Static.is_dead && ~Analysis_Done
            if Dynamic.x >= (Static.death_km * 1000)
                Dyn_Arrival_Time = k * dt;
                Dyn_Batt_At_Checkpoint = Dynamic.batt;
                Analysis_Done = true;
            end
        end
        
        if Dynamic.batt <= 0, Dynamic.is_dead = true; Dynamic.death_km = Dynamic.x/1000; end
        if Dynamic.x >= Target_Distance_m, Dynamic.finished = true; end
    end
    
    % LOGGING
    if mod(k, Step) == 0
        Log_Ind = Log_Ind + 1;
        
        L_Dist(Log_Ind) = Dynamic.x / 1000; 
        L_Soc_S(Log_Ind) = max(0, (Static.batt/Battery_Cap_kWh)*100);
        L_Soc_D(Log_Ind) = max(0, (Dynamic.batt/Battery_Cap_kWh)*100);
        
        L_Time(Log_Ind) = k * dt;
        L_Gap_S(Log_Ind) = x_lead(k) - Static.x;
        L_Gap_D(Log_Ind) = x_lead(k) - Dynamic.x;
        L_Vel_S(Log_Ind) = Static.v * 3.6;
        L_Vel_D(Log_Ind) = Dynamic.v * 3.6;
        L_Vel_L(Log_Ind) = v_lead(k) * 3.6; 
    end
    
    if Static.is_dead && Dynamic.finished, break; end 
    if Static.is_dead && Dynamic.is_dead, break; end 
    k = k + 1;
end

% TRIM ARRAYS
L_Dist=L_Dist(1:Log_Ind); L_Soc_S=L_Soc_S(1:Log_Ind); L_Soc_D=L_Soc_D(1:Log_Ind);
L_Time=L_Time(1:Log_Ind);
L_Gap_S=L_Gap_S(1:Log_Ind); L_Gap_D=L_Gap_D(1:Log_Ind);
L_Vel_S=L_Vel_S(1:Log_Ind); L_Vel_D=L_Vel_D(1:Log_Ind); L_Vel_L=L_Vel_L(1:Log_Ind);

%% 4. FINAL REPORT
fprintf('\n=================================================\n');
fprintf('           TESLA-STYLE DEATH POINT ANALYSIS      \n');
fprintf('=================================================\n');

% 1. DEATH POINT ANALYSIS
fprintf('1. DEATH POINT ANALYSIS:\n');
if Static.is_dead
    fprintf('   Static Car:  Battery died at %.2f km (STRANDED).\n', Static.death_km);
    
    if Analysis_Done
        Delay = Dyn_Arrival_Time - Static.death_time;
        fprintf('   Dynamic Car: Passed the same point only %.1f seconds later.\n', Delay);
        fprintf('   >> DYNAMIC SOC AT THAT MOMENT: %% %.1f\n', (Dyn_Batt_At_Checkpoint/Battery_Cap_kWh)*100);
    else
        fprintf('   Dynamic car has not reached that point yet.\n');
    end
else
    fprintf('   Static car miraculously finished.\n');
end
fprintf('\n');

% 2. RANGE AND CONSUMPTION
Eff_S = (Battery_Cap_kWh - Static.batt) / (Static.x/1000) * 100;
Eff_D = (Battery_Cap_kWh - Dynamic.batt) / (Dynamic.x/1000) * 100;

fprintf('2. CONSUMPTION AND RANGE:\n');
fprintf('   Static Consumption:  %.2f kWh/100km  (Range: %.0f km)\n', Eff_S, Static.x/1000);
fprintf('   Dynamic Consumption: %.2f kWh/100km  (Range: %.0f km)\n', Eff_D, Dynamic.x/1000);
fprintf('   >> EFFICIENCY GAIN:  %% %.1f Better Efficiency\n', ((Eff_S - Eff_D)/Eff_S)*100);
fprintf('\n');

% 3. REGEN BREAKDOWN
fprintf('3. BRAKING ENERGY ANALYSIS:\n');
fprintf('   Static Car:\n');
fprintf('     - Recovered (Battery): %.2f kWh\n', Static.regen);
fprintf('     - Wasted (Heat):       %.2f kWh (WASTE)\n', Static.waste);
fprintf('   Dynamic Car:\n');
fprintf('     - Recovered (Battery): %.2f kWh\n', Dynamic.regen);
fprintf('     - Wasted (Heat):       %.2f kWh (Perfect)\n', Dynamic.waste);
fprintf('=================================================\n');

%% 5. VISUALIZATION (FIGURE 1: DASHBOARD)
figure(1); set(gcf, 'Color','w', 'Position', [50, 50, 1200, 900]);

% A. Battery Depletion
subplot(3,2,1);
plot(L_Dist, L_Soc_S, 'r', 'LineWidth', 2); hold on;
plot(L_Dist, L_Soc_D, 'b', 'LineWidth', 2);
if Static.is_dead
    plot(Static.death_km, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 3);
end
xlabel('Distance (km)'); ylabel('SoC (%)');
title('Battery Depletion'); legend('Static', 'Dynamic'); grid on; ylim([0 100]);

% B. Consumption (kWh/100km)
subplot(3,2,2);
bar([Eff_S, Eff_D], 'FaceColor', 'flat');
xticklabels({'Static', 'Dynamic'});
ylabel('kWh / 100 km'); title('Average Consumption'); grid on;

% C. Regen Energy
subplot(3,2,3);
bar([Static.regen, Dynamic.regen], 'FaceColor', 'flat');
xticklabels({'Static', 'Dynamic'});
ylabel('kWh'); title('Recovered Energy (Regen)'); grid on;

% D. Wasted Energy
subplot(3,2,4);
bar([Static.waste, Dynamic.waste], 'FaceColor', 'flat');
xticklabels({'Static', 'Dynamic'});
ylabel('kWh'); title('Brake Heat Loss (Waste)'); grid on;

% E. Distance to Leader
subplot(3,2,5);
plot(L_Time/60, L_Gap_S, 'r'); hold on;
plot(L_Time/60, L_Gap_D, 'b');
yline(0, 'k');
xlabel('Time (min)'); ylabel('Distance (m)');
title('Distance to Leader (Gap)');
legend('Static (Tight)', 'Dynamic (Elastic)'); grid on; ylim([0 600]);

% F. Average Speed
subplot(3,2,6);
Avg_S = mean(L_Vel_S(L_Vel_S > 1)); 
Avg_D = mean(L_Vel_D(L_Vel_D > 1));

b_avg = bar([Avg_S, Avg_D], 'FaceColor', 'flat');
b_avg.CData(1,:) = [1 0 0]; b_avg.CData(2,:) = [0 0 1];
xticklabels({'Static', 'Dynamic'});
ylabel('km/h'); title('Average Speed');
text(1, Avg_S, sprintf('%.1f', Avg_S), 'Horiz','center','Vert','bottom');
text(2, Avg_D, sprintf('%.1f', Avg_D), 'Horiz','center','Vert','bottom');
grid on;

%% 6. NEW VISUALIZATION (FIGURE 2: SPEED PROFILE)
figure(2); set(gcf, 'Name', 'Speed Profile', 'Color', 'w', 'Position', [150, 150, 1200, 600]);

plot(L_Time/60, L_Vel_L, 'Color', [0.7 0.7 0.7], 'LineWidth', 1); hold on; % Leader (Grey)
plot(L_Time/60, L_Vel_S, 'r', 'LineWidth', 1.5); % Static (Red)
plot(L_Time/60, L_Vel_D, 'b', 'LineWidth', 2);   % Dynamic (Blue)

xlabel('Time (minutes)'); ylabel('Speed (km/h)');
title('Velocity Profile Comparison: Lead vs Static vs Dynamic');
legend('Leader (Traffic)', 'Static (Aggressive)', 'Dynamic (Eco-Smoothed)');
grid on;


%% PHYSICS ENGINE (TESLA BLENDED BRAKING)
function [v, x, b, w_kWh, r_kWh] = phys_step(v, x, b, a, dt, m, rho, cd, A, crr, g, aux, max_regen_kw)
    if b <= 0, v=0; w_kWh=0; r_kWh=0; return; end
    a = max(-8, min(4.0, a));
    v_new = v + a * dt; if v_new < 0, v_new = 0; end
    x_new = x + v_new * dt;
    
    F_tot = (m*a) + (0.5*rho*cd*A*v^2) + (v>0.1)*(crr*m*g);
    P_shaft = F_tot * v; 
    
    w_kWh = 0; r_kWh = 0;
    
    if P_shaft > 0 
        % CONSUMPTION
        eff = 0.90; 
        P_e = (P_shaft / eff) + (aux*1000);
        b = b - (P_e * dt / 3.6e6);
    else 
        % BRAKING (Blended)
        P_req = abs(P_shaft);
        Limit_W = max_regen_kw * 1000;
        
        if P_req > Limit_W
            P_regen = Limit_W;          % Take up to limit
            P_waste = P_req - Limit_W;  % Rest is heat
            w_kWh = (P_waste * dt) / 3.6e6; 
        else
            P_regen = P_req;            % Take all
            P_waste = 0;
        end
        
        % Generator Efficiency and Net Charge
        P_chg = (P_regen * 0.85) - (aux*1000);
        if P_chg > 0
            e_gain = (P_chg * dt / 3.6e6);
            b = b + e_gain;
            r_kWh = e_gain; % NET RECHARGE
        else
            b = b - (abs(P_chg) * dt / 3.6e6);
        end
    end
    
    % --- TESLA SPEED LIMITER (Performance Model) ---
    max_speed_ms = 250 / 3.6; % 250 km/h
    if v_new > max_speed_ms
        v_new = max_speed_ms; 
    end
    
    v = v_new; x = x_new;
end