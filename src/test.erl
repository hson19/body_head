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
    io:format("State prediction: ~p~n", [State_prediction]),
    P0 = mat:eye(46),
    T1 = hera:timestamp(),
    
    %% Generate a random list of length 46 and call the EKF predict function
    {_, _} = e4:ekf_predict({State_prediction, P0}, {fun e4:state_prediction/1, fun e4:jacobian_state_prediction/1}, mat:eye(46)),
    
    T2 = hera:timestamp(),
    io:format("Time taken for ekf_predict: ~p microseconds~n", [T2 - T1]),
    T2-T1.
ekf_update_time_tracking()->
    init_random(),
    State_prediction = generate_random_list(46, 0, 100),
    Z_random=generate_random_list(46, 0, 100),
    P0 = mat:eye(46),
    T1 = hera:timestamp(),
    
    Hxp = generate_random_list(46, 0, 100),
    H = generate_random_matrix(46, 0, 100),
    R=generate_random_matrix(46, 0, 100),

    S = mat:eval([H, '*', P0, '*´', H, '+', R]),
    io:format("S: ~p~n", [S]),
    %% Generate a random list of length 46 and call the EKF predict function
    {_, _} = e4:ekf_update({State_prediction, P0}, {Hxp,H }, R,Z_random),
    
    T2 = hera:timestamp(),
    io:format("Time taken for ekf_update: ~p microseconds~n", [T2 - T1]),
    T2-T1.

