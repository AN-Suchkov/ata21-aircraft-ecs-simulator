@echo off
setlocal

if not exist logs mkdir logs

set BIN=.\skv_sim_21.exe

echo === skv_sim_21 — запуск всех сценариев ===

%BIN% scenarios\scenario_default.csv                logs\log_default.csv
%BIN% scenarios\scenario_hot_day.csv                logs\log_hot_day.csv
%BIN% scenarios\scenario_extreme_cold_start.csv     logs\log_extreme_cold.csv
%BIN% scenarios\scenario_pid_step_response.csv      logs\log_pid_step.csv
%BIN% scenarios\scenario_anti_ice_test.csv          logs\log_anti_ice.csv
%BIN% scenarios\scenario_line_break_and_recovery.csv logs\log_line_break.csv
%BIN% scenarios\scenario_ground_test.csv            logs\log_ground_test.csv
%BIN% scenarios\scenario_dual_fault.csv             logs\log_dual_fault.csv
%BIN% scenarios\scenario_channel_fault.csv          logs\log_channel_fault.csv
%BIN% scenarios\scenario_sensor_power_loss.csv      logs\log_sensor_power.csv
%BIN% scenarios\scenario_ch_disagr.csv              logs\log_ch_disagr.csv
%BIN% scenarios\scenario_temp_high.csv              logs\log_temp_high.csv
%BIN% scenarios\scenario_both_ch_fault.csv          logs\log_both_ch_fault.csv

echo === Готово. Логи в logs\ ===
endlocal
