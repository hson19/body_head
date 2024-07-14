-module(test).

%% Include necessary headers (if they contain macros or record definitions)
% -include("e1.hrl").
% -include("e2.hrl").
% -include("e3.hrl").
% -include("e4.erl").

-export([ekf_update_time_tracking/0]).
-export([ekf_predict_time_tracking/0]).
-define(STATE_KEYS, [hq0, hq1, hq2, hq3, hpx, hpy, hpz, h_vx, h_vy, h_vz,
                    h_ax, h_ay, h_az, h_gx, h_gy, h_gz, bq0, bq1, bq2, bq3,
                    bpx, bpy, bpz, gBpx, gBpy, gBpz, aq0, aq1, aq2, aq3,
                    apx, apy, apz, gApx, gApy, gApz, fq0, fq1, fq2, fq3,
                    fpx, fpy, fpz, gFpx, gFpy, gFpz,time]).
-export([ekf_predict_time_tracking_loop/1]).
-export([ekf_update_time_tracking_loop/1]).

%% Initialize the random number generator
init_random() ->
    rand:seed(exsplus, 41).


generate_random_list(Length, Min, Max) ->

    generate_random_list_helper(Length, Min, Max, []).

generate_random_list_helper(0, _Min, _Max, Acc) ->
    Acc;
generate_random_list_helper(N, Min, Max, Acc) ->
    RandomInt =  Min + (rand:uniform() * (Max - Min)),
    RandomIntList= [RandomInt],
    generate_random_list_helper(N - 1, Min, Max, [RandomIntList | Acc]).

generate_random_list_1D(Length, Min, Max) ->
    generate_random_list_helper_1D(Length, Min, Max, []).

generate_random_list_helper_1D(0, _Min, _Max, Acc) ->
    Acc;
generate_random_list_helper_1D(N, Min, Max, Acc) ->
    RandomInt =  Min + (rand:uniform() * (Max - Min)),
    generate_random_list_helper_1D(N - 1, Min, Max, [RandomInt | Acc]).

generate_random_matrix(Length, Min, Max) ->
    generate_random_matrix_helper(Length,Length, Min, Max, []).
generate_random_matrix_helper(0,_Length, _Min, _Max, Acc) ->
    Acc;
generate_random_matrix_helper(N,Length, Min, Max, Acc) ->
    RandomRow = generate_random_list_1D(Length, Min, Max),
    generate_random_matrix_helper(N - 1,Length, Min, Max, [RandomRow | Acc]).
%% Main function to track EKF update time
ekf_predict_time_tracking() ->
    init_random(),
    State_prediction = generate_random_list(47, 0, 100),
    P0 = mat:eye(46),
    %% Generate a random list of length 46 and call the EKF predict function
    {_, _} = e4:ekf_predict({State_prediction, P0}, {fun e4:state_prediction/1, fun e4:jacobian_state_prediction/1}, mat:eye(46)).
ekf_update_time_tracking()->
    init_random(),
    State_prediction = mat:matrix(generate_random_list(46, 0, 100)),
    Z_random=mat:matrix(generate_random_list(46, 0, 100)),
    P0 = mat:eye(46),
    
    Hxp = mat:matrix(generate_random_list(46, 0, 100)),
    H = mat:matrix(generate_random_matrix(46, 0, 100)),
    R=mat:matrix(generate_random_matrix(46, 0, 100)),

    %% Generate a random list of length 46 and call the EKF predict function
    {Time,_} = timer:tc(e4, ekf_update, [{State_prediction, P0}, {Hxp,H }, R,Z_random]),
    % {Time, _} = timer:tc(e4:ekf_update({State_prediction, P0}, {Hxp,H }, R,Z_random)),
    Time.

ekf_predict_time_tracking_loop(N) ->
    % Call N times the ekf_predict_time_tracking function
    %  and keeps a list of time taken
    %  returns the mean of the list
    ekf_predict_time_tracking_loop_helper(N, 0, []).
ekf_predict_time_tracking_loop_helper(0, Sum, Acc) ->
    Mean_time_in_microsecond=erlang:floor(Sum/ length(Acc)),
    Total_time_in_microsecond=Sum,

    Total_time_second=Total_time_in_microsecond/1000000,
    Mean_time_in_second=Mean_time_in_microsecond/1000000,
    io:format("Mean Time in second: ~p~n", [Mean_time_in_second]),
    io:format("Total Time in second: ~p~n", [Total_time_second]);
ekf_predict_time_tracking_loop_helper(N, Sum, Acc) ->
    {Time,_} = timer:tc(fun ekf_predict_time_tracking/0), % Time in micro seconds
    Time_seconds=Time,
    ekf_predict_time_tracking_loop_helper(N - 1, Sum + Time_seconds, [Time_seconds | Acc]).

ekf_update_time_tracking_loop(N) ->
    % Call N times the ekf_predict_time_tracking function
    %  and keeps a list of time taken
    %  returns the mean of the list
    ekf_update_time_tracking_loop_helper(N, 0, []).
ekf_update_time_tracking_loop_helper(0, Sum, Acc) ->
    Mean_time_in_microsecond=erlang:floor(Sum/ length(Acc)),
    Total_time_in_microsecond=Sum,

    Total_time_second=Total_time_in_microsecond/1000000,
    Mean_time_in_second=Mean_time_in_microsecond/1000000,
    io:format("Mean Time in second: ~p~n", [Mean_time_in_second]),
    io:format("Total Time in second: ~p~n", [Total_time_second]);
ekf_update_time_tracking_loop_helper(N, Sum, Acc) ->
    Time = ekf_update_time_tracking(),
    ekf_update_time_tracking_loop_helper(N - 1, Sum + Time, [Time | Acc]).


