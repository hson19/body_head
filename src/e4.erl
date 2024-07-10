-module(e4).

-behaviour(hera_measure).

-export([calibrate/1]).
-export([init/1, measure/1]).
-export([observationModel/3]).
-export([jacobian_state_prediction/1]).
-export([state_prediction/1]).
-export([ekf_predict/3]).
-export([ekf_update/4]).

-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).
-define(Debug_info, true).
-define(BODYHEADDISTANCE, 0.30).
-define(HEADSHOULDERDISTANCE, 0.25).
-define(SHOULDERBICEPSDISTANCE, 0.15).
-define(BICEPSELBOWDISTANCE, 0.15).
-define(ELBOWWRISTDISTANCE, 0.15).
-define(STATE_KEYS, [hq0, hq1, hq2, hq3, hpx, hpy, hpz, h_vx, h_vy, h_vz,
                    h_ax, h_ay, h_az, h_gx, h_gy, h_gz, bq0, bq1, bq2, bq3,
                    bpx, bpy, bpz, gBpx, gBpy, gBpz, aq0, aq1, aq2, aq3,
                    apx, apy, apz, gApx, gApy, gApz, fq0, fq1, fq2, fq3,
                    fpx, fpy, fpz, gFpx, gFpy, gFpz,time]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate({MBx,MBy,MBz}) ->
    _ = io:get_line("Place the pmod_nav at 0° then press enter"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 100),
    [Mx,My,Mz] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    R0 = ahrs([Ax,Ay,-Az], [-(Mx-MBx),My-MBy,-(Mz-MBz)]), % why minus? ?? THe z axis is inverted on the sensor
    io:format("Calibration matrice at calibration e2: ~p~n", [mat:tr(R0)]),
    mat:tr(R0).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(R0) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 100
    },
    T0 = hera:timestamp(),
    Head_pos=[[0],[0],[0]], % Position Head
    Head_v=[[0],[0],[0]], % V Head
    Head_a=[[0],[0],[0]], % Acc Head
    Head_quat= dcm2quat(R0),

    
    Body_pos = quatTransfomation(Head_quat,[[-?BODYHEADDISTANCE],[0],[0]]),
    Body_quat= dcm2quat(R0),

    Arm_pos = quatTransfomation(Head_quat,[[-?HEADSHOULDERDISTANCE],[-?SHOULDERBICEPSDISTANCE],[0]]),
    Arm_quat= dcm2quat(R0),
    
    Forearm_pos= quatTransfomation(Head_quat,[[-?HEADSHOULDERDISTANCE],[-?SHOULDERBICEPSDISTANCE-?BICEPSELBOWDISTANCE-?ELBOWWRISTDISTANCE],[0]]),
    Forearm_quat= dcm2quat(R0),

    P0=mat:eye(46),
    State = {Head_quat++Head_pos++Head_v++Head_a++[[0],[0],[0]]++Body_quat++Body_pos++[[0],[0],[0]]++Arm_quat++Arm_pos++[[0],[0],[0]]++Forearm_quat++Forearm_pos++[[0],[0],[0]]++[[T0]],P0},
    {ok, State, Spec}.


measure({State,P0}) ->
    % TODO : check if we receive measurement
    % TODO : if there is measurement prediction step followed by measurement update
    DataBody = hera_data:get(nav3, body_head@body),
    DataHead = hera_data:get(nav3, body_head@head),
    DataArm = hera_data:get(nav3, body_head@arm),
    Dataforearm = hera_data:get(nav3, body_head@forearm),

    State_dict=state_to_state_dict(State),
    T0=maps:get(time, state_to_state_dict(State)),
    T1 = hera:timestamp(),
    NavHead = [Data || {_,_,Ts,Data} <- DataHead, T0 < Ts, T1-Ts < 500],    
    NavBody = [Data || {_,_,Ts,Data} <- DataBody, T0 < Ts, T1-Ts < 500],
    NavArm = [Data || {_,_,Ts,Data} <- DataArm, T0 < Ts, T1-Ts < 500],
    NavForearm = [Data || {_,_,Ts,Data} <- Dataforearm, T0 < Ts, T1-Ts < 500],
    if length(NavHead) == 0 andalso length(NavBody) == 0 andalso length(NavArm) == 0 andalso length(NavForearm) == 0 -> 
        {undefined, {State, P0}};
    true ->
    Dt= (T1-maps:get(time,State_dict))/1000,
    State_dict_dt=maps:put(time, Dt, State_dict), % change the time to dt
    State_prediction=state_dict_to_state(State_dict_dt),
    StartPrediction=hera:timestamp(),
    {New_state,P1}=ekf_predict({State_prediction,P0}, {fun state_prediction/1,fun jacobian_state_prediction/1}, mat:eye(46)),
    EndPrediction=hera:timestamp(),
    Dt_prediction=EndPrediction-StartPrediction,

    New_state_time=New_state++[[T1]],
    New_state_dict=state_to_state_dict(New_state_time),
    Init_z=[], % initial measurement
    Init_h=[], % initial measurement jacobian
    {Head_z,Head_h}=measurement_head(NavHead,New_state_dict,Init_z,Init_h),
    {Body_z,Body_h}=measurement_body(NavBody,New_state_dict,Head_z,Head_h),
    {Arm_z,Arm_h}=measurement_arm(NavArm,New_state_dict,Body_z,Body_h),
    {Z,H}=measurement_forearm(NavForearm,New_state_dict,Arm_z,Arm_h),
    Hxp=mat:'*'(H,New_state),
    StartUpdate=hera:timestamp(),
    {Updated_state,P2}=ekf_update({New_state,P1}, {Hxp,H},mat:eye(length(Z)),Z),
    EndUpdate=hera:timestamp(),
    Dt_update=EndUpdate-StartUpdate,
    Final_state=Updated_state++[[T1]],
    Final_state_normalized=normalize_quat_state(Final_state),
    {ok,[Dt_prediction,Dt_update],{Final_state_normalized,P2}}
    end.
    

            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize state_dict
getNewestSensorData([], [], [], []) ->
    io:format("All data lists are empty~n"),
    {[], none};
getNewestSensorData(Head_data, Body_data, Arm_data, Forearm_data) ->
    DataLists = [[head]++Head_data, [body]++Body_data, [arm]++Arm_data, [forearm]++Forearm_data],
    % Ensure each list has at least one element and the first element is a number (T1)
    ValidDataLists = lists:filter(fun
        (Data) when is_list(Data), length(Data) > 1-> true;
        (_) -> false
    end, DataLists),
    io:format("ValidDataLists: ~p~n", [ValidDataLists]),
    case ValidDataLists of
        [] ->
            io:format("All data lists are empty or invalid~n"),
            {[], none};
        _ ->
            % Convert lists to tuples {T1, Data}
            Tuples = lists:map(fun([Card |[[T1 | Rest]]]) -> {T1, Rest,Card} end, ValidDataLists),
            io:format("Tuples: ~p~n", [Tuples]),
            % Find the tuple with the smallest T1
            MinTuple = lists:max(Tuples),
            io:format("MinTuple: ~p~n", [MinTuple]),
            io:format("Head_data: ~p~n", [Head_data]),

            % Match MinTuple to the corresponding data and return
            case MinTuple of
            {T1, Data, Card} when Card == head -> {Data, head};
            {T1, Data, Card} when Card == body -> {Data, body};
            {T1, Data, Card} when Card == arm -> {Data, arm};
            {T1, Data, Card} when Card == forearm -> {Data, forearm};
            {T1, Data, Card} -> {Data, none}
            end
    end.

state_to_state_dict(Values) ->
    % List of initial values corresponding to the keys
    lists:foldl(fun({Key, [Value]}, Acc) ->
                    maps:put(Key, Value, Acc)
                end, #{}, lists:zip(?STATE_KEYS, Values)).
state_dict_to_state(State_dict) ->
    lists:map(fun(Key) -> [maps:get(Key, State_dict)] end, ?STATE_KEYS).
state_prediction(State)->
    State_dict=state_to_state_dict(State),
    Head_quat = [[maps:get(hq0, State_dict)], [maps:get(hq1, State_dict)], [maps:get(hq2, State_dict)], [maps:get(hq3, State_dict)]],
    Head_pos = [[maps:get(hpx, State_dict)], [maps:get(hpy, State_dict)], [maps:get(hpz, State_dict)]],
    Head_v = [[maps:get(h_vx, State_dict)], [maps:get(h_vy, State_dict)], [maps:get(h_vz, State_dict)]],
    Head_a = [[maps:get(h_ax, State_dict)], [maps:get(h_ay, State_dict)], [maps:get(h_az, State_dict)]],
    Head_g = [[maps:get(h_gx, State_dict)], [maps:get(h_gy, State_dict)], [maps:get(h_gz, State_dict)]],
    Body_quat = [[maps:get(bq0, State_dict)], [maps:get(bq1, State_dict)], [maps:get(bq2, State_dict)], [maps:get(bq3, State_dict)]],
    % Body_pos = [maps:get(bpx, State_dict), maps:get(bpy, State_dict), maps:get(bpz, State_dict)],
    Body_g = [[maps:get(gBpx, State_dict)], [maps:get(gBpy, State_dict)], [maps:get(gBpz, State_dict)]],
    Arm_quat = [[maps:get(aq0, State_dict)], [maps:get(aq1, State_dict)], [maps:get(aq2, State_dict)], [maps:get(aq3, State_dict)]],
    % Arm_pos = [maps:get(apx, State_dict), maps:get(apy, State_dict), maps:get(apz, State_dict)],
    Arm_g = [[maps:get(gApx, State_dict)], [maps:get(gApy, State_dict)], [maps:get(gApz, State_dict)]],
    Forearm_quat = [[maps:get(fq0, State_dict)], [maps:get(fq1, State_dict)], [maps:get(fq2, State_dict)], [maps:get(fq3, State_dict)]],
    % Forearm_pos = [maps:get(fpx, State_dict), maps:get(fpy, State_dict), maps:get(fpz, State_dict)],
    Forearm_g = [[maps:get(gFpx, State_dict)], [maps:get(gFpy, State_dict)], [maps:get(gFpz, State_dict)]],
    Dt = maps:get(time, State_dict),
    
    % Head prediction
    {ok,New_head_quat} = kalman_quaternion_predict(Head_g, Dt, Head_quat, mat:eye(4)),
    {ok,New_head_pos,New_head_v,_} = kalman_position_predict(Dt, Head_quat,Head_pos,Head_v,Head_a, mat:eye(9)), % don't need the new ax ?
    % Body prediction
    {ok,New_body_quat} = kalman_quaternion_predict(Body_g, Dt, Body_quat, mat:eye(4)),
    New_body_pos = mat:'+'(quatTransfomation(Head_quat,[[0],[-?BODYHEADDISTANCE],[0]]),Head_pos),
    % Arm prediction
    io:format("Arm_g: ~p~n", [Arm_g]),
    io:format("Arm_quat: ~p~n", [Arm_quat]),
    io:format("Dt: ~p~n", [Dt]),
    {ok,New_arm_quat} = kalman_quaternion_predict(Arm_g, Dt, Arm_quat, mat:eye(4)),
    Shoulder_pos = mat:'+'(quatTransfomation(Head_quat,[[0],[-?HEADSHOULDERDISTANCE],[0]]),Head_pos),
    Shoulder_to_biceps = quatTransfomation(Arm_quat,[[-?SHOULDERBICEPSDISTANCE],[0],[0]]),
    New_arm_pos = mat:'+'(Shoulder_pos,Shoulder_to_biceps),
    % Forearm prediction
    {ok,New_forearm_quat} = kalman_quaternion_predict(Forearm_g, Dt, Forearm_quat, mat:eye(4)),
    Elbow_pos = mat:'+'(quatTransfomation(Arm_quat,[[-?BICEPSELBOWDISTANCE],[0],[0]]),Shoulder_pos),
    New_forearm_pos = mat:'+'(Elbow_pos,quatTransfomation(Forearm_quat,[[-?ELBOWWRISTDISTANCE],[0],[0]])),
    % Create a list of new values
    New_values = New_head_quat++New_head_pos++New_head_v++Head_a++Head_g++
                                New_body_quat++New_body_pos++Body_g++
                                New_arm_quat++New_arm_pos++Arm_g++
                                New_forearm_quat++New_forearm_pos++Forearm_g,
    New_values.


jacobian_state_prediction(State) ->
    % Here the time is DT

    State_dict_dt=state_to_state_dict(State),
    Dt=maps:get(time, State_dict_dt),
    % Create a list of zeros and put one at the Head position
    Ntoid = maps:from_list(lists:zip(?STATE_KEYS, lists:seq(0, length(?STATE_KEYS) - 1))),
    [HpNorth,HpUP,HpEast,VpNorth,VpUP,VpEast] = get_position_pred_matrix(Dt, State_dict_dt),
    % Initialize the position matrices with zeros and set specific values
    NKeys_with_time = length(maps:keys(State_dict_dt)),
    NKeys= NKeys_with_time-1,
    H=mat:zeros(NKeys,NKeys),
    H1 = set_element(maps:get(hpx, Ntoid) + 1, H, HpNorth),
    H2 = set_element(maps:get(hpy, Ntoid) + 1, H1, HpUP),
    H3 = set_element(maps:get(hpz, Ntoid) + 1, H2, HpEast),
    H4 = set_element(maps:get(h_vx, Ntoid) + 1, H3, VpNorth),
    H5 = set_element(maps:get(h_vy, Ntoid) + 1, H4, VpUP),
    H6 = set_element(maps:get(h_vz, Ntoid) + 1, H5, VpEast),
    
    % Quat Transformation for Head
    Quat_pred_line_10=lists:duplicate(NKeys,0),
    Quat_pred_line_11=set_element(maps:get(hq1, Ntoid) + 1, Quat_pred_line_10, maps:get(h_gx, State_dict_dt)),
    Quat_pred_line_12=set_element(maps:get(hq2, Ntoid) + 1, Quat_pred_line_11, maps:get(h_gy, State_dict_dt)),
    Quat_pred_line_13=set_element(maps:get(hq3, Ntoid) + 1, Quat_pred_line_12, maps:get(h_gz, State_dict_dt)),
    H7 = set_element(maps:get(hq0, Ntoid) + 1, H6, Quat_pred_line_13),

    Quat_pred_line_20=lists:duplicate(NKeys,0),
    Quat_pred_line_21=set_element(maps:get(hq0, Ntoid) + 1, Quat_pred_line_20, -maps:get(h_gx, State_dict_dt)),
    Quat_pred_line_22=set_element(maps:get(hq2, Ntoid) + 1, Quat_pred_line_21, -maps:get(h_gz, State_dict_dt)),
    Quat_pred_line_23=set_element(maps:get(hq3, Ntoid) + 1, Quat_pred_line_22, maps:get(h_gy, State_dict_dt)),
    H8 = set_element(maps:get(hq1, Ntoid) + 1, H7, Quat_pred_line_23),

    Quat_pred_line_30=lists:duplicate(NKeys,0),
    Quat_pred_line_31=set_element(maps:get(hq0, Ntoid) + 1, Quat_pred_line_30, -maps:get(h_gy, State_dict_dt)),
    Quat_pred_line_32=set_element(maps:get(hq1, Ntoid) + 1, Quat_pred_line_31, maps:get(h_gz, State_dict_dt)),
    Quat_pred_line_33=set_element(maps:get(hq3, Ntoid) + 1, Quat_pred_line_32, -maps:get(h_gx, State_dict_dt)),
    H9 = set_element(maps:get(hq2, Ntoid) + 1, H8, Quat_pred_line_33),

    Quat_pred_line_40=lists:duplicate(NKeys,0),
    Quat_pred_line_41=set_element(maps:get(hq0, Ntoid) + 1, Quat_pred_line_40, -maps:get(h_gz, State_dict_dt)),
    Quat_pred_line_42=set_element(maps:get(hq1, Ntoid) + 1, Quat_pred_line_41, -maps:get(h_gy, State_dict_dt)),
    Quat_pred_line_43=set_element(maps:get(hq2, Ntoid) + 1, Quat_pred_line_42, maps:get(h_gx, State_dict_dt)),
    H10 = set_element(maps:get(hq3, Ntoid) + 1, H9, Quat_pred_line_43),

    H11 = set_element(maps:get(h_gx, Ntoid) + 1, H10, set_element(maps:get(h_gx,Ntoid),lists:duplicate(NKeys,0),1)),
    H12 = set_element(maps:get(h_gy, Ntoid) + 1, H11, set_element(maps:get(h_gy,Ntoid),lists:duplicate(NKeys,0),1)),
    H13 = set_element(maps:get(h_gz, Ntoid) + 1, H12, set_element(maps:get(h_gz,Ntoid),lists:duplicate(NKeys,0),1)),
    H14 = set_element(maps:get(h_ax, Ntoid) + 1, H13, set_element(maps:get(h_ax,Ntoid),lists:duplicate(NKeys,0),1)),
    H15 = set_element(maps:get(h_ay, Ntoid) + 1, H14, set_element(maps:get(h_ay,Ntoid),lists:duplicate(NKeys,0),1)),
    H16 = set_element(maps:get(h_az, Ntoid) + 1, H15, set_element(maps:get(h_az,Ntoid),lists:duplicate(NKeys,0),1)),

    {Body_x,Body_y,Body_z,Arm_x,Arm_y,Arm_z,Forearm_x,Forearm_y,Forearm_z}=get_jacobian_position_matrix(State_dict_dt, Ntoid),

    H17 = set_element(maps:get(bpx, Ntoid) + 1, H16, Body_x),
    H18 = set_element(maps:get(bpy, Ntoid) + 1, H17, Body_y),
    H19 = set_element(maps:get(bpz, Ntoid) + 1, H18, Body_z),
    H20 = set_element(maps:get(apx, Ntoid) + 1, H19, Arm_x),
    H21 = set_element(maps:get(apy, Ntoid) + 1, H20, Arm_y),
    H22 = set_element(maps:get(apz, Ntoid) + 1, H21, Arm_z),
    H23 = set_element(maps:get(fpx, Ntoid) + 1, H22, Forearm_x),
    H24 = set_element(maps:get(fpy, Ntoid) + 1, H23, Forearm_y),
    H25 = set_element(maps:get(fpz, Ntoid) + 1, H24, Forearm_z),

    % Set 1 for acceleration and gyro for Body, Arm and Forearm

    H26=set_element(maps:get(gBpx, Ntoid) + 1, H25, set_element(maps:get(gBpx,Ntoid),lists:duplicate(NKeys,0),1)),
    H27=set_element(maps:get(gBpy, Ntoid) + 1, H26, set_element(maps:get(gBpy,Ntoid),lists:duplicate(NKeys,0),1)),
    H28=set_element(maps:get(gBpz, Ntoid) + 1, H27, set_element(maps:get(gBpz,Ntoid),lists:duplicate(NKeys,0),1)),
    H29=set_element(maps:get(gApx, Ntoid) + 1, H28, set_element(maps:get(gApx,Ntoid),lists:duplicate(NKeys,0),1)),
    H30=set_element(maps:get(gApy, Ntoid) + 1, H29, set_element(maps:get(gApy,Ntoid),lists:duplicate(NKeys,0),1)),
    H31=set_element(maps:get(gApz, Ntoid) + 1, H30, set_element(maps:get(gApz,Ntoid),lists:duplicate(NKeys,0),1)),
    H32=set_element(maps:get(gFpx, Ntoid) + 1, H31, set_element(maps:get(gFpx,Ntoid),lists:duplicate(NKeys,0),1)),
    H33=set_element(maps:get(gFpy, Ntoid) + 1, H32, set_element(maps:get(gFpy,Ntoid),lists:duplicate(NKeys,0),1)),
    H34=set_element(maps:get(gFpz, Ntoid) + 1, H33, set_element(maps:get(gFpz,Ntoid),lists:duplicate(NKeys,0),1)),

    % Quat Transformation Arm 

    Quat_pred_line_arm_10=lists:duplicate(NKeys,0),
    Quat_pred_line_arm_11=set_element(maps:get(aq1, Ntoid) + 1, Quat_pred_line_arm_10, maps:get(gApx, State_dict_dt)),
    Quat_pred_line_arm_12=set_element(maps:get(aq2, Ntoid) + 1, Quat_pred_line_arm_11, maps:get(gApy, State_dict_dt)),
    Quat_pred_line_arm_13=set_element(maps:get(aq3, Ntoid) + 1, Quat_pred_line_arm_12, maps:get(gApz, State_dict_dt)),
    H35 = set_element(maps:get(aq0, Ntoid) + 1, H34, Quat_pred_line_arm_13),

    Quat_pred_line_arm_20=lists:duplicate(NKeys,0),
    Quat_pred_line_arm_21=set_element(maps:get(aq0, Ntoid) + 1, Quat_pred_line_arm_20, -maps:get(gApx, State_dict_dt)),
    Quat_pred_line_arm_22=set_element(maps:get(aq2, Ntoid) + 1, Quat_pred_line_arm_21, -maps:get(gApz, State_dict_dt)),
    Quat_pred_line_arm_23=set_element(maps:get(aq3, Ntoid) + 1, Quat_pred_line_arm_22, maps:get(gApy, State_dict_dt)),
    H36 = set_element(maps:get(aq1, Ntoid) + 1, H35, Quat_pred_line_arm_23),

    Quat_pred_line_arm_30=lists:duplicate(NKeys,0),
    Quat_pred_line_arm_31=set_element(maps:get(aq0, Ntoid) + 1, Quat_pred_line_arm_30, -maps:get(gApy, State_dict_dt)),
    Quat_pred_line_arm_32=set_element(maps:get(aq1, Ntoid) + 1, Quat_pred_line_arm_31, maps:get(gApz, State_dict_dt)),
    Quat_pred_line_arm_33=set_element(maps:get(aq3, Ntoid) + 1, Quat_pred_line_arm_32, -maps:get(gApx, State_dict_dt)),
    H37 = set_element(maps:get(aq2, Ntoid) + 1, H36, Quat_pred_line_arm_33),

    Quat_pred_line_arm_40=lists:duplicate(NKeys,0),
    Quat_pred_line_arm_41=set_element(maps:get(aq0, Ntoid) + 1, Quat_pred_line_arm_40, -maps:get(gApz, State_dict_dt)),
    Quat_pred_line_arm_42=set_element(maps:get(aq1, Ntoid) + 1, Quat_pred_line_arm_41, -maps:get(gApy, State_dict_dt)),
    Quat_pred_line_arm_43=set_element(maps:get(aq2, Ntoid) + 1, Quat_pred_line_arm_42, maps:get(gApx, State_dict_dt)),
    H38 = set_element(maps:get(aq3, Ntoid) + 1, H37, Quat_pred_line_arm_43),

    % Quat Transformation Forearm

    Quat_pred_line_forearm_10=lists:duplicate(NKeys,0),
    Quat_pred_line_forearm_11=set_element(maps:get(fq1, Ntoid) + 1, Quat_pred_line_forearm_10, maps:get(gFpx, State_dict_dt)),
    Quat_pred_line_forearm_12=set_element(maps:get(fq2, Ntoid) + 1, Quat_pred_line_forearm_11, maps:get(gFpy, State_dict_dt)),
    Quat_pred_line_forearm_13=set_element(maps:get(fq3, Ntoid) + 1, Quat_pred_line_forearm_12, maps:get(gFpz, State_dict_dt)),
    H39 = set_element(maps:get(fq0, Ntoid) + 1, H38, Quat_pred_line_forearm_13),

    Quat_pred_line_forearm_20=lists:duplicate(NKeys,0),
    Quat_pred_line_forearm_21=set_element(maps:get(fq0, Ntoid) + 1, Quat_pred_line_forearm_20, -maps:get(gFpx, State_dict_dt)),
    Quat_pred_line_forearm_22=set_element(maps:get(fq2, Ntoid) + 1, Quat_pred_line_forearm_21, -maps:get(gFpz, State_dict_dt)),
    Quat_pred_line_forearm_23=set_element(maps:get(fq3, Ntoid) + 1, Quat_pred_line_forearm_22, maps:get(gFpy, State_dict_dt)),
    H40 = set_element(maps:get(fq1, Ntoid) + 1, H39, Quat_pred_line_forearm_23),

    Quat_pred_line_forearm_30=lists:duplicate(NKeys,0),
    Quat_pred_line_forearm_31=set_element(maps:get(fq0, Ntoid) + 1, Quat_pred_line_forearm_30, -maps:get(gFpy, State_dict_dt)),
    Quat_pred_line_forearm_32=set_element(maps:get(fq1, Ntoid) + 1, Quat_pred_line_forearm_31, maps:get(gFpz, State_dict_dt)),
    Quat_pred_line_forearm_33=set_element(maps:get(fq3, Ntoid) + 1, Quat_pred_line_forearm_32, -maps:get(gFpx, State_dict_dt)),
    H41 = set_element(maps:get(fq2, Ntoid) + 1, H40, Quat_pred_line_forearm_33),

    Quat_pred_line_forearm_40=lists:duplicate(NKeys,0),
    Quat_pred_line_forearm_41=set_element(maps:get(fq0, Ntoid) + 1, Quat_pred_line_forearm_40, -maps:get(gFpz, State_dict_dt)),
    Quat_pred_line_forearm_42=set_element(maps:get(fq1, Ntoid) + 1, Quat_pred_line_forearm_41, -maps:get(gFpy, State_dict_dt)),
    Quat_pred_line_forearm_43=set_element(maps:get(fq2, Ntoid) + 1, Quat_pred_line_forearm_42, maps:get(gFpx, State_dict_dt)),
    H42 = set_element(maps:get(fq3, Ntoid) + 1, H41, Quat_pred_line_forearm_43),

    % Quat Transformation for Body
    Quat_pred_line_Body_10=lists:duplicate(NKeys,0),
    Quat_pred_line_Body_11=set_element(maps:get(bq1, Ntoid) + 1, Quat_pred_line_Body_10, maps:get(gBpx, State_dict_dt)),
    Quat_pred_line_Body_12=set_element(maps:get(bq2, Ntoid) + 1, Quat_pred_line_Body_11, maps:get(gBpy, State_dict_dt)),
    Quat_pred_line_Body_13=set_element(maps:get(bq3, Ntoid) + 1, Quat_pred_line_Body_12, maps:get(gBpz, State_dict_dt)),
    H43 = set_element(maps:get(bq0, Ntoid) + 1, H42, Quat_pred_line_Body_13),

    Quat_pred_line_Body_20=lists:duplicate(NKeys,0),
    Quat_pred_line_Body_21=set_element(maps:get(bq0, Ntoid) + 1, Quat_pred_line_Body_20, -maps:get(gBpx, State_dict_dt)),
    Quat_pred_line_Body_22=set_element(maps:get(bq2, Ntoid) + 1, Quat_pred_line_Body_21, -maps:get(gBpz, State_dict_dt)),
    Quat_pred_line_Body_23=set_element(maps:get(bq3, Ntoid) + 1, Quat_pred_line_Body_22, maps:get(gBpy, State_dict_dt)),
    H44 = set_element(maps:get(bq1, Ntoid) + 1, H43, Quat_pred_line_Body_23),

    Quat_pred_line_Body_30=lists:duplicate(NKeys,0),
    Quat_pred_line_Body_31=set_element(maps:get(bq0, Ntoid) + 1, Quat_pred_line_Body_30, -maps:get(gBpy, State_dict_dt)),
    Quat_pred_line_Body_32=set_element(maps:get(bq1, Ntoid) + 1, Quat_pred_line_Body_31, maps:get(gBpz, State_dict_dt)),
    Quat_pred_line_Body_33=set_element(maps:get(bq3, Ntoid) + 1, Quat_pred_line_Body_32, -maps:get(gBpx, State_dict_dt)),
    H45 = set_element(maps:get(bq2, Ntoid) + 1, H44, Quat_pred_line_Body_33),

    Quat_pred_line_Body_40=lists:duplicate(NKeys,0),
    Quat_pred_line_Body_41=set_element(maps:get(bq0, Ntoid) + 1, Quat_pred_line_Body_40, -maps:get(gBpz, State_dict_dt)),
    Quat_pred_line_Body_42=set_element(maps:get(bq1, Ntoid) + 1, Quat_pred_line_Body_41, -maps:get(gBpy, State_dict_dt)),
    Quat_pred_line_Body_43=set_element(maps:get(bq2, Ntoid) + 1, Quat_pred_line_Body_42, maps:get(gBpx, State_dict_dt)),
    H46 = set_element(maps:get(bq3, Ntoid) + 1, H45, Quat_pred_line_Body_43),
    H46.

get_position_pred_matrix(Dt, StateDict) ->
    % Extract quaternion and acceleration components from the state dictionary
    Ntoid = maps:from_list(lists:zip(?STATE_KEYS, lists:seq(0, length(?STATE_KEYS) - 1))),
    Q0 = maps:get(hq0, StateDict),
    Q1 = maps:get(hq1, StateDict),
    Q2 = maps:get(hq2, StateDict),
    Q3 = maps:get(hq3, StateDict),
    Ax = maps:get(h_ax, StateDict),
    Ay = maps:get(h_ay, StateDict),
    Az = maps:get(h_az, StateDict),

    % Compute derivatives for 'north'
    DaNorthDQ0 = 4 * Q0 * Ax - 2 * Q3 * Ay - 2 * Q2 * Az,
    DaNorthDQ1 = 4 * Q1 * Ax + 2 * Q2 * Ay + 2 * Q3 * Az,
    DaNorthDQ2 = 2 * Q1 * Ay + 2 * Q0 * Az,
    DaNorthDQ3 = -2 * Q0 * Ay + 2 * Q1 * Az,
    DaNorthDAx = 2 * Q0 * Q0 + 2 * Q1 * Q1 - 1,
    DaNorthDAy = 2 * Q1 * Q2 - 2 * Q0 * Q3,
    DaNorthDAz = 2 * Q1 * Q3 + 2 * Q0 * Q2,

    % Compute derivatives for 'up'
    DaUpDQ0 = 2 * Q3 * Ax + 4 * Q0 * Ay - 2 * Q1 * Az,
    DaUpDQ1 = 2 * Q2 * Ax - 2 * Q0 * Az,
    DaUpDQ2 = 2 * Q1 * Ax + 4 * Q2 * Ay + 2 * Q3 * Az,
    DaUpDQ3 = 2 * Q0 * Ax + 2 * Q2 * Az,
    DaUpDAx = 2 * Q1 * Q2 + 2 * Q0 * Q3,
    DaUpDAy = 2 * (Q0 * Q0 + Q2 * Q2 - 1),
    DaUpDAz = 2 * Q2 * Q3 - 2 * Q0 * Q1,

    % Compute derivatives for 'east'
    DaEastDQ0 = -2 * Q2 * Ax + 2 * Q1 * Ay + 4 * Q0 * Az,
    DaEastDQ1 = 2 * Q3 * Ax + 2 * Q0 * Ay,
    DaEastDQ2 = -2 * Q0 * Ax + 2 * Q3 * Ay,
    DaEastDQ3 = 2 * Q1 * Ax + 2 * Q2 * Ay + 4 * Q3 * Az,
    DaEastDAx = 2 * Q1 * Q3 - 2 * Q0 * Q2,
    DaEastDAy = 2 * Q2 * Q3 + 2 * Q0 * Q1,
    DaEastDAz = 2 * Q0 * Q0 + 2 * Q3 * Q3 - 1,

    % Initialize the position matrices with zeros and set specific values
    NKeys = length(maps:keys(StateDict))-1,

    PNorth = lists:duplicate(NKeys, 0),
    PNorth1 = set_element(maps:get(hpx, Ntoid) + 1, PNorth, 1),
    PNorth2 = set_element(maps:get(h_vx, Ntoid) + 1, PNorth1, Dt),
    PNorth3 = set_element(maps:get(hq0, Ntoid) + 1, PNorth2, DaNorthDQ0 * Dt),
    PNorth4 = set_element(maps:get(hq1, Ntoid) + 1, PNorth3, DaNorthDQ1 * Dt),
    PNorth5 = set_element(maps:get(hq2, Ntoid) + 1, PNorth4, DaNorthDQ2 * Dt),
    PNorth6 = set_element(maps:get(hq3, Ntoid) + 1, PNorth5, DaNorthDQ3 * Dt),
    PNorth7 = set_element(maps:get(h_ax, Ntoid) + 1, PNorth6, DaNorthDAx * Dt),
    PNorth8 = set_element(maps:get(h_ay, Ntoid) + 1, PNorth7, DaNorthDAy * Dt),
    PNorth9 = set_element(maps:get(h_az, Ntoid) + 1, PNorth8, DaNorthDAz * Dt),

    PUp = lists:duplicate(NKeys, 0),
    PUp1 = set_element(maps:get(hpy, Ntoid) + 1, PUp, 1),
    PUp2 = set_element(maps:get(h_vy, Ntoid) + 1, PUp1, Dt),
    PUp3 = set_element(maps:get(hq0, Ntoid) + 1, PUp2, DaUpDQ0 * Dt),
    PUp4 = set_element(maps:get(hq1, Ntoid) + 1, PUp3, DaUpDQ1 * Dt),
    PUp5 = set_element(maps:get(hq2, Ntoid) + 1, PUp4, DaUpDQ2 * Dt),
    PUp6 = set_element(maps:get(hq3, Ntoid) + 1, PUp5, DaUpDQ3 * Dt),
    PUp7 = set_element(maps:get(h_ax, Ntoid) + 1, PUp6, DaUpDAx * Dt),
    PUp8 = set_element(maps:get(h_ay, Ntoid) + 1, PUp7, DaUpDAy * Dt),
    PUp9 = set_element(maps:get(h_az, Ntoid) + 1, PUp8, DaUpDAz * Dt),

    PEast = lists:duplicate(NKeys, 0),
    PEast1 = set_element(maps:get(hpz, Ntoid) + 1, PEast, 1),
    PEast2 = set_element(maps:get(h_vz, Ntoid) + 1, PEast1, Dt),
    PEast3 = set_element(maps:get(hq0, Ntoid) + 1, PEast2, DaEastDQ0 * Dt),
    PEast4 = set_element(maps:get(hq1, Ntoid) + 1, PEast3, DaEastDQ1 * Dt),
    PEast5 = set_element(maps:get(hq2, Ntoid) + 1, PEast4, DaEastDQ2 * Dt),
    PEast6 = set_element(maps:get(hq3, Ntoid) + 1, PEast5, DaEastDQ3 * Dt),
    PEast7 = set_element(maps:get(h_ax, Ntoid) + 1, PEast6, DaEastDAx * Dt),
    PEast8 = set_element(maps:get(h_ay, Ntoid) + 1, PEast7, DaEastDAy * Dt),
    PEast9 = set_element(maps:get(h_az, Ntoid) + 1, PEast8, DaEastDAz * Dt),

    VNorth =lists:duplicate(NKeys, 0),
    VNorth1 = set_element(maps:get(h_vx, Ntoid) + 1, VNorth, 1),
    VNorth2 = set_element(maps:get(hq0, Ntoid) + 1, VNorth1, DaNorthDQ0 * Dt),
    VNorth3 = set_element(maps:get(hq1, Ntoid) + 1, VNorth2, DaNorthDQ1 * Dt),
    VNorth4 = set_element(maps:get(hq2, Ntoid) + 1, VNorth3, DaNorthDQ2 * Dt),
    VNorth5 = set_element(maps:get(hq3, Ntoid) + 1, VNorth4, DaNorthDQ3 * Dt),
    VNorth6 = set_element(maps:get(h_ax, Ntoid) + 1, VNorth5, DaNorthDAx * Dt),
    VNorth7 = set_element(maps:get(h_ay, Ntoid) + 1, VNorth6, DaNorthDAy * Dt),
    VNorth8 = set_element(maps:get(h_az, Ntoid) + 1, VNorth7, DaNorthDAz * Dt),

    VUp =lists:duplicate(NKeys, 0),
    VUp1 = set_element(maps:get(h_vy, Ntoid) + 1, VUp, 1),
    VUp2 = set_element(maps:get(hq0, Ntoid) + 1, VUp1, DaUpDQ0 * Dt),
    VUp3 = set_element(maps:get(hq1, Ntoid) + 1, VUp2, DaUpDQ1 * Dt),
    VUp4 = set_element(maps:get(hq2, Ntoid) + 1, VUp3, DaUpDQ2 * Dt),
    VUp5 = set_element(maps:get(hq3, Ntoid) + 1, VUp4, DaUpDQ3 * Dt),
    VUp6 = set_element(maps:get(h_ax, Ntoid) + 1, VUp5, DaUpDAx * Dt),
    VUp7 = set_element(maps:get(h_ay, Ntoid) + 1, VUp6, DaUpDAy * Dt),
    VUp8 = set_element(maps:get(h_az, Ntoid) + 1, VUp7, DaUpDAz * Dt),

    VEast = lists:duplicate(NKeys, 0),
    VEast1 = set_element(maps:get(h_vz, Ntoid) + 1, VEast, 1),
    VEast2 = set_element(maps:get(hq0, Ntoid) + 1, VEast1, DaEastDQ0 * Dt),
    VEast3 = set_element(maps:get(hq1, Ntoid) + 1, VEast2, DaEastDQ1 * Dt),
    VEast4 = set_element(maps:get(hq2, Ntoid) + 1, VEast3, DaEastDQ2 * Dt),
    VEast5 = set_element(maps:get(hq3, Ntoid) + 1, VEast4, DaEastDQ3 * Dt),
    VEast6 = set_element(maps:get(h_ax, Ntoid) + 1, VEast5, DaEastDAx * Dt),
    VEast7 = set_element(maps:get(h_ay, Ntoid) + 1, VEast6, DaEastDAy * Dt),
    VEast8 = set_element(maps:get(h_az, Ntoid) + 1, VEast7, DaEastDAz * Dt),
    

    [PNorth9, PUp9, PEast9, VNorth8, VUp8, VEast8].
get_jacobian_position_matrix( StateDict, Ntoid) ->
    % TODO should be addition not set for forearm
    N = length(maps:keys(StateDict))-1,

    % Body X Position
    BodyXPosition = lists:duplicate(N, 0),
    BodyXPosition1 = set_element(maps:get(hq0, Ntoid) + 1, BodyXPosition, -0.6 * maps:get(hq0, StateDict)),
    BodyXPosition2 = set_element(maps:get(hq1, Ntoid) + 1, BodyXPosition1, -0.6 * maps:get(hq1, StateDict)),
    BodyXPosition3 = set_element(maps:get(hpx, Ntoid) + 1, BodyXPosition2, 1),

    % Body Y Position
    BodyYPosition = lists:duplicate(N, 0),
    BodyYPosition1 = set_element(maps:get(hpy, Ntoid) + 1, BodyYPosition, 1),
    BodyYPosition2 = set_element(maps:get(hq0, Ntoid) + 1, BodyYPosition1, -0.3 * maps:get(hq3, StateDict)),
    BodyYPosition3 = set_element(maps:get(hq1, Ntoid) + 1, BodyYPosition2, -0.3 * maps:get(hq2, StateDict)),
    BodyYPosition4 = set_element(maps:get(hq2, Ntoid) + 1, BodyYPosition3, -0.3 * maps:get(hq1, StateDict)),
    BodyYPosition5 = set_element(maps:get(hq3, Ntoid) + 1, BodyYPosition4, -0.3 * maps:get(hq0, StateDict)),

    % Body Z Position
    BodyZPosition = lists:duplicate(N, 0),
    BodyZPosition1 = set_element(maps:get(hpz, Ntoid) + 1, BodyZPosition, 1),
    BodyZPosition2 = set_element(maps:get(hq0, Ntoid) + 1, BodyZPosition1, 0.3 * maps:get(hq2, StateDict)),
    BodyZPosition3 = set_element(maps:get(hq1, Ntoid) + 1, BodyZPosition2, -0.3 * maps:get(hq3, StateDict)),
    BodyZPosition4 = set_element(maps:get(hq2, Ntoid) + 1, BodyZPosition3, 0.3 * maps:get(hq0, StateDict)),
    BodyZPosition5 = set_element(maps:get(hq3, Ntoid) + 1, BodyZPosition4, -0.3 * maps:get(hq1, StateDict)),

    % Arm X Position
    ArmXPosition = lists:duplicate(N, 0),
    ArmXPosition1 = set_element(maps:get(hpx, Ntoid) + 1, ArmXPosition, 1),
    ArmXPosition2 = set_element(maps:get(hq0, Ntoid) + 1, ArmXPosition1, 0.3 * maps:get(hq3, StateDict)),
    ArmXPosition3 = set_element(maps:get(hq1, Ntoid) + 1, ArmXPosition2, -0.3 * maps:get(hq2, StateDict)),
    ArmXPosition4 = set_element(maps:get(hq2, Ntoid) + 1, ArmXPosition3, -0.3 * maps:get(hq1, StateDict)),
    ArmXPosition5 = set_element(maps:get(hq3, Ntoid) + 1, ArmXPosition4, 0.3 * maps:get(hq0, StateDict)),
    ArmXPosition6 = set_element(maps:get(aq0, Ntoid) + 1, ArmXPosition5, -0.6 * maps:get(aq0, StateDict)),
    ArmXPosition7 = set_element(maps:get(aq1, Ntoid) + 1, ArmXPosition6, -0.6 * maps:get(aq1, StateDict)),

    % Arm Y Position
    ArmYPosition = lists:duplicate(N, 0),
    ArmYPosition1 = set_element(maps:get(hpy, Ntoid) + 1, ArmYPosition, 1),
    ArmYPosition2 = set_element(maps:get(hq0, Ntoid) + 1, ArmYPosition1, -0.6 * maps:get(hq0, StateDict)),
    ArmYPosition3 = set_element(maps:get(hq2, Ntoid) + 1, ArmYPosition2, -0.6 * maps:get(hq2, StateDict)),
    ArmYPosition4 = set_element(maps:get(aq0, Ntoid) + 1, ArmYPosition3, -0.3 * maps:get(aq3, StateDict)),
    ArmYPosition5 = set_element(maps:get(aq1, Ntoid) + 1, ArmYPosition4, -0.3 * maps:get(aq2, StateDict)),
    ArmYPosition6 = set_element(maps:get(aq2, Ntoid) + 1, ArmYPosition5, -0.3 * maps:get(aq1, StateDict)),
    ArmYPosition7 = set_element(maps:get(aq3, Ntoid) + 1, ArmYPosition6, -0.3 * maps:get(aq0, StateDict)),

    % Arm Z Position
    ArmZPosition = lists:duplicate(N, 0),
    ArmZPosition1 = set_element(maps:get(hpz, Ntoid) + 1, ArmZPosition, 1),
    ArmZPosition2 = set_element(maps:get(hq0, Ntoid) + 1, ArmZPosition1, -0.3 * maps:get(hq1, StateDict)),
    ArmZPosition3 = set_element(maps:get(hq1, Ntoid) + 1, ArmZPosition2, -0.3 * maps:get(hq0, StateDict)),
    ArmZPosition4 = set_element(maps:get(hq2, Ntoid) + 1, ArmZPosition3, -0.3 * maps:get(hq3, StateDict)),
    ArmZPosition5 = set_element(maps:get(hq3, Ntoid) + 1, ArmZPosition4, -0.3 * maps:get(hq2, StateDict)),
    ArmZPosition6 = set_element(maps:get(aq0, Ntoid) + 1, ArmZPosition5, 0.3 * maps:get(aq2, StateDict)),
    ArmZPosition7 = set_element(maps:get(aq1, Ntoid) + 1, ArmZPosition6, -0.3 * maps:get(aq3, StateDict)),
    ArmZPosition8 = set_element(maps:get(aq2, Ntoid) + 1, ArmZPosition7, 0.3 * maps:get(aq0, StateDict)),
    ArmZPosition9 = set_element(maps:get(aq3, Ntoid) + 1, ArmZPosition8, -0.3 * maps:get(aq1, StateDict)),

    % Forearm X Position
    ForearmXPosition = ArmXPosition6,
    ForearmXPosition1 = set_element(maps:get(aq0, Ntoid) + 1, ForearmXPosition, -0.6 * maps:get(aq0, StateDict)),
    ForearmXPosition2 = set_element(maps:get(aq1, Ntoid) + 1, ForearmXPosition1, -0.6 * maps:get(aq1, StateDict)),
    ForearmXPosition3 = set_element(maps:get(fq0, Ntoid) + 1, ForearmXPosition2, -0.6 * maps:get(fq0, StateDict)),
    ForearmXPosition4 = set_element(maps:get(fq1, Ntoid) + 1, ForearmXPosition3, -0.6 * maps:get(fq1, StateDict)),

    % Forearm Y Position
    ForearmYPosition = ArmYPosition7,
    ForearmYPosition1 = set_element(maps:get(aq0, Ntoid) + 1, ForearmYPosition, -0.3 * maps:get(aq3, StateDict)),
    ForearmYPosition2 = set_element(maps:get(aq1, Ntoid) + 1, ForearmYPosition1, -0.3 * maps:get(aq2, StateDict)),
    ForearmYPosition3 = set_element(maps:get(aq2, Ntoid) + 1, ForearmYPosition2, -0.3 * maps:get(aq1, StateDict)),
    ForearmYPosition4 = set_element(maps:get(aq3, Ntoid) + 1, ForearmYPosition3, -0.3 * maps:get(aq0, StateDict)),
    ForearmYPosition5 = set_element(maps:get(fq0, Ntoid) + 1, ForearmYPosition4, -0.3 * maps:get(fq3, StateDict)),
    ForearmYPosition6 = set_element(maps:get(fq1, Ntoid) + 1, ForearmYPosition5, -0.3 * maps:get(fq2, StateDict)),
    ForearmYPosition7 = set_element(maps:get(fq2, Ntoid) + 1, ForearmYPosition6, -0.3 * maps:get(fq1, StateDict)),
    ForearmYPosition8 = set_element(maps:get(fq3, Ntoid) + 1, ForearmYPosition7, -0.3 * maps:get(fq0, StateDict)),

    % Forearm Z Position
    ForearmZPosition = ArmZPosition9,
    ForearmZPosition1 = set_element(maps:get(aq0, Ntoid) + 1, ForearmZPosition, 0.3 * maps:get(aq2, StateDict)),
    ForearmZPosition2 = set_element(maps:get(aq1, Ntoid) + 1, ForearmZPosition1, -0.3 * maps:get(aq3, StateDict)),
    ForearmZPosition3 = set_element(maps:get(aq2, Ntoid) + 1, ForearmZPosition2, 0.3 * maps:get(aq0, StateDict)),
    ForearmZPosition4 = set_element(maps:get(aq3, Ntoid) + 1, ForearmZPosition3, -0.3 * maps:get(aq1, StateDict)),
    ForearmZPosition5 = set_element(maps:get(fq0, Ntoid) + 1, ForearmZPosition4, 0.3 * maps:get(fq2, StateDict)),
    ForearmZPosition6 = set_element(maps:get(fq1, Ntoid) + 1, ForearmZPosition5, -0.3 * maps:get(fq3, StateDict)),
    ForearmZPosition7 = set_element(maps:get(fq2, Ntoid) + 1, ForearmZPosition6, 0.3 * maps:get(fq0, StateDict)),
    ForearmZPosition8 = set_element(maps:get(fq3, Ntoid) + 1, ForearmZPosition7, -0.3 * maps:get(fq1, StateDict)),

    {BodyXPosition3, BodyYPosition5, BodyZPosition5, ArmXPosition7,
     ArmYPosition7, ArmZPosition9, ForearmXPosition4, ForearmYPosition8, ForearmZPosition8}.

state_measurement(State) ->
    H=jacobian_state_measurement(State),
    State_without_time=lists:sublist(State,46),
    Result=mat:'*'(H,State_without_time),
    Result.
jacobian_state_measurement(State) ->
    H=mat:eye(46),
    H.
head_state_measurement(State)-> 
    H=jacobian_head_state_measurement(State),
    State_without_time=lists:sublist(State,46),
    Result=mat:'*'(H,State_without_time),
    Result.
jacobian_head_state_measurement(State) ->
      H=[ create_list_with_one_at(0,46),
        create_list_with_one_at(1,46),
        create_list_with_one_at(2,46),
        create_list_with_one_at(3,46),
        create_list_with_one_at(10,46),
        create_list_with_one_at(11,46),
        create_list_with_one_at(12,46),
        create_list_with_one_at(13,46),
        create_list_with_one_at(14,46),
        create_list_with_one_at(15,46)
    ],
    H.  
body_state_measurement(State) ->
    H=jacobian_body_state_measurement(State),
    State_without_time=lists:sublist(State,46),
    Result=mat:'*'(H,State_without_time),
    Result.
jacobian_body_state_measurement(State) ->
    H=[ create_list_with_one_at(16,46),
        create_list_with_one_at(17,46),
        create_list_with_one_at(18,46),
        create_list_with_one_at(19,46),
        create_list_with_one_at(23,46),
        create_list_with_one_at(24,46),
        create_list_with_one_at(25,46)
    ],
    H.
arm_state_measurement(State) ->
    H=jacobian_arm_state_measurement(State),
    State_without_time=lists:sublist(State,46),
    Result=mat:'*'(H,State_without_time),
    Result.
jacobian_arm_state_measurement(State) ->
    H=[ create_list_with_one_at(26,46),
        create_list_with_one_at(27,46),
        create_list_with_one_at(28,46),
        create_list_with_one_at(29,46),
        create_list_with_one_at(33,46),
        create_list_with_one_at(34,46),
        create_list_with_one_at(35,46)
    ],
    H.

forearm_state_measurement(State) ->
    H=jacobian_forearm_state_measurement(State),
    State_without_time=lists:sublist(State,46),
    Result=mat:'*'(H,State_without_time),
    Result.
jacobian_forearm_state_measurement(State) ->
    H=[ create_list_with_one_at(36,46),
        create_list_with_one_at(37,46),
        create_list_with_one_at(38,46),
        create_list_with_one_at(39,46),
        create_list_with_one_at(43,46),
        create_list_with_one_at(44,46),
        create_list_with_one_at(45,46)
    ],
    H.
kalman_quaternion_predict(Gyro,Dt,Quat,P0) ->
    Conj_quat = conjugateQuaternion(Quat),
    [[Wx],[Wy],[Wz]] = Gyro,
    Omega =[ 
        [0,Wx,Wy,Wz],
        [-Wx,0,-Wz,Wy],
        [-Wy,Wz,0,-Wx],
        [-Wz,-Wy,Wx,0]
    ],
    F = mat:'+'(mat:eye(4), mat:'*'(0.5*Dt, Omega)),
    Q = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
    {Xorp, _} = kalman:kf_predict({Conj_quat,P0}, F, Q),

    UnitXorp = unit(conjugateQuaternion(Xorp)),
    io:format("predicted quat: ~p~n", [UnitXorp]),
    {ok,UnitXorp}.
kalman_position_predict(Dt,Quat,[[X],[Y],[Z]],[[Vx],[Vy],[Vz]],Acc,P0) ->
    % state have quaternion, position, velocity and acceleration
    [[Ax],[Ay],[Az]]= quatTransfomation(Quat,Acc), % TODO chehck if conjugate or not ??
    State = [[X],[Vx],[Ax],[Y],[Vy],[Ay],[Z],[Vz],[Az]],
    F = [
        [1,Dt,(Dt*Dt)/2,0,0,0,0,0,0], % North
        [0,1,Dt,0,0,0,0,0,0], % V_North
        [0,0,1,0,0,0,0,0,0], % Acc_North
        [0,0,0,1,Dt,(Dt*Dt)/2,0,0,0], % UP
        [0,0,0,0,1,Dt,0,0,0], % V_UP
        [0,0,0,0,0,1,0,0,0], % Acc_UP
        [0,0,0,0,0,0,1,Dt,(Dt*Dt)/2], % EAST
        [0,0,0,0,0,0,0,1,Dt], % V_EAST
        [0,0,0,0,0,0,0,0,1] % Acc_EAST
    ],
    Q = mat:diag([?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL]),
    {[[New_x],[New_vx],[New_ax],[New_y],[New_vy],[New_ay],[New_z],[New_vz],[New_az]], _} = kalman:kf_predict({State,P0}, F, Q),
    {ok, [[New_x],[New_y],[New_z]],[[New_vx],[New_vy],[New_vz]],[[New_ax],[New_ay],[New_az]]}.
identity_prediction(State) ->
    lists:sublist(State,46).
jacobian_identity_prediction(State) ->
    mat:eye(46).
measurement_head(Nav_data,State_dict_dt,Current_z,Current_h) ->   
    State_prediction=state_dict_to_state(State_dict_dt),
    io:format("Head_nav_data: ~p~n", [Nav_data]),
    io:format("length: ~p~n", [length(Nav_data)]),
    if length(Nav_data) == 0 -> 
        io:format("No data head"),
        {Current_z,Current_h};
    true ->
        {Acc, Gyro, Mag}=process_nav(hd(Nav_data)),
        [Ax,Ay,Az]=Acc,
        [Gx,Gy,Gz]=Gyro,
        Qhead=[[maps:get(hq0, State_dict_dt)], [maps:get(hq1, State_dict_dt)], [maps:get(hq2, State_dict_dt)], [maps:get(hq3, State_dict_dt)]],
        ObservedQuat=observationModel(Qhead,Acc,Mag),
        
        case qdot(ObservedQuat, Qhead) > 0 of
        true ->
            Qc= ObservedQuat;
        false ->
            Qc= mat:'*'(-1,ObservedQuat)            
        end,
        Z=Current_z++Qc++[[Ax],[Ay],[Az]]++[[Gx],[Gy],[Gz]],
        H=Current_h++jacobian_head_state_measurement(State_dict_dt),
        {Z,H}
    end.
measurement_body(Nav_data,State_dict_dt,Current_z,Current_h) ->
    State_prediction=state_dict_to_state(State_dict_dt),
    if length(Nav_data) == 0 -> 
        {Current_z,Current_h};
    true ->
        {Acc, Gyro, Mag}=process_nav(hd(Nav_data)),
        [Gx,Gy,Gz]=Gyro,
        Qbody=[[maps:get(bq0, State_dict_dt)], [maps:get(bq1, State_dict_dt)], [maps:get(bq2, State_dict_dt)], [maps:get(bq3, State_dict_dt)]],
        ObservedQuat=observationModel(Qbody,Acc,Mag),
        case qdot(ObservedQuat, Qbody) > 0 of
        true ->
            Qc= ObservedQuat;
        false ->
            Qc= mat:'*'(-1,ObservedQuat)            
        end,
        Z=Current_z++Qc++[[Gx],[Gy],[Gz]],
        H=Current_h++jacobian_body_state_measurement(State_dict_dt),
        {Z,H}
    end.   
measurement_arm(Nav_data,State_dict_dt,Current_z,Current_h) ->
    State_prediction=state_dict_to_state(State_dict_dt),
    if length(Nav_data) == 0 -> 
        {Current_z,Current_h};
    true ->
        {Acc, Gyro, Mag}=process_nav(hd(Nav_data)),
        [Gx,Gy,Gz]=Gyro,
        Qarm=[[maps:get(aq0, State_dict_dt)], [maps:get(aq1, State_dict_dt)], [maps:get(aq2, State_dict_dt)], [maps:get(aq3, State_dict_dt)]],
        ObservedQuat=observationModel(Qarm,Acc,Mag),
        case qdot(ObservedQuat, Qarm) > 0 of
        true ->
            Qc= ObservedQuat;
        false ->
            Qc= mat:'*'(-1,ObservedQuat)            
        end,
        Z=Current_z++Qc++[[Gx],[Gy],[Gz]],
        H=Current_h++jacobian_arm_state_measurement(State_dict_dt),
        {Z,H}
    end.
measurement_forearm(Nav_data,State_dict_dt,Current_z,Current_h) ->
    State_prediction=state_dict_to_state(State_dict_dt),
    if length(Nav_data) == 0 -> 
        {Current_z,Current_h};
    true ->
        {Acc, Gyro, Mag}=process_nav(hd(Nav_data)),
        [Gx,Gy,Gz]=Gyro,        
        Qforearm=[[maps:get(fq0, State_dict_dt)], [maps:get(fq1, State_dict_dt)], [maps:get(fq2, State_dict_dt)], [maps:get(fq3, State_dict_dt)]],
        ObservedQuat=observationModel(Qforearm,Acc,Mag),
        case qdot(ObservedQuat, Qforearm) > 0 of
        true ->
            Qc= ObservedQuat;
        false ->
            Qc= mat:'*'(-1,ObservedQuat)            
        end,
        Z=Current_z++Qc++[[Gx],[Gy],[Gz]],
        H=Current_h++jacobian_forearm_state_measurement(State_dict_dt),
        {Z,H}
    end.

qdot([[Q11], [Q12], [Q13], [Q14]], [[Q21], [Q22], [Q23], [Q24]]) ->
    Q11*Q21 + Q12*Q22 + Q13*Q23 + Q14*Q24.
dot([[X1],[X2],[X3]], [[Y1],[Y2],[Y3]]) ->
    X1*Y1 + X2*Y2 + X3*Y3.
consttimesVector(C, [[X],[Y],[Z]]) ->
    [[C*X], [C*Y], [C*Z]];
consttimesVector(C, [X,Y,Z]) ->
    [C*X, C*Y, C*Z].
process_nav([]) -> 
    {[],[],[],[]};
process_nav(Nav) ->
    {Acc, Next} = lists:split(3, Nav),
    {Gyro, Mag} = lists:split(3, Next),
    {Acc, Gyro, Mag}. 

unit([[W],[X],[Y],[Z]])->
    Norm = math:sqrt(W*W + X*X + Y*Y + Z*Z),
    [[W/Norm],[X/Norm],[Y/Norm],[Z/Norm]];
unit([[X],[Y],[Z]]) ->
    Norm = math:sqrt(X*X + Y*Y + Z*Z),
    [[X/Norm],[Y/Norm],[Z/Norm]];
unit(V) ->
    Norm = math:sqrt(lists:sum([X*X || X <- V])),
    [X/Norm || X <- V].


scale(List, Factor) ->
    [X*Factor || X <- List].


calibrate(Comp, Registers, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers)) || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].


ahrs(Acc, Mag) ->
    UP = unit([A || A <-Acc]),
    Down = unit([-A || A <- Acc]),
    East = unit(cross_product(Down, unit(Mag))),
    North = unit(cross_product(East, Down)),
    mat:tr([North, UP, East]).


cross_product([[U1],[U2],[U3]], [[V1],[V2],[V3]]) -> 
    [[U2*V3-U3*V2], [U3*V1-U1*V3], [U1*V2-U2*V1]];
cross_product([U1,U2,U3], [V1,V2,V3]) -> 
    [U2*V3-U3*V2, U3*V1-U1*V3, U1*V2-U2*V1].
q_product([[Q11],[Q12],[Q13],[Q14]],[[Q21],[Q22],[Q23],[Q24]]) -> %TODO CHECK
    [[Q11*Q21-Q12*Q22-Q13*Q23-Q14*Q24],
     [Q11*Q22+Q12*Q21+Q13*Q24-Q14*Q23],
     [Q11*Q23-Q12*Q24+Q13*Q21+Q14*Q22],
     [Q11*Q24+Q12*Q23-Q13*Q22+Q14*Q21]].

dcm2quat(R) ->
    [[R11,R12,R13],
     [R21,R22,R23],
     [R31,R32,R33]
    ] = R,
    Q12 = 0.25*(1+R11+R22+R33),
    Q1 = math:sqrt(Q12),
    V = [
        4*Q12,
        R32-R23,
        R13-R31,
        R21-R12
    ],
    NewV=scale(V, (0.25/Q1)),
    [W,X,Y,Z]=NewV,
    [[W],[X],[Y],[Z]]. % return the quaternion

q2dcm([[Q0], [Q1], [Q2], [Q3]]) -> 
    R00 = 2 * (Q0 * Q0 + Q1 * Q1) - 1,
    R01 = 2 * (Q1 * Q2 - Q0 * Q3),
    R02 = 2 * (Q1 * Q3 + Q0 * Q2),
     
    R10 = 2 * (Q1 * Q2 + Q0 * Q3),
    R11 = 2 * (Q0 * Q0 + Q2 * Q2) - 1,
    R12 = 2 * (Q2 * Q3 - Q0 * Q1),
     
    R20 = 2 * (Q1 * Q3 - Q0 * Q2),
    R21 = 2 * (Q2 * Q3 + Q0 * Q1),
    R22 = 2 * (Q0 * Q0 + Q3 * Q3) - 1,

    [[R00, R01, R02],
     [R10, R11, R12],
     [R20, R21, R22]].

inverseQuatTransformation([[Q0],[Q1],[Q2],[Q3]],Fp) ->
    % F(p) = Q*p*Q^-1 <=> p = Q^-1*F(p)*Q
    Qinv = [[Q0],[-Q1],[-Q2],[-Q3]],
    Q = [[Q0],[Q1],[Q2],[Q3]],
    P = q_product(Qinv,q_product(Fp,Q)),
    P.
quatTransfomation([[Q0],[Q1],[Q2],[Q3]],P)->
    % F(p) = Q*p*Q^-1 
    Qinv = [[Q0],[-Q1],[-Q2],[-Q3]],
    Q = [[Q0],[Q1],[Q2],[Q3]],
    P_quat = [[0]|P],
    Fp= q_product(Q,q_product(P_quat,Qinv)),
    % returns the 3 last element of Fp
    lists:nthtail(1,Fp).

observationModel(Qn,Acc,Mag) ->
    % transform quaterion to matrice
    Mua = 1, %0.005, %TODO FIND VALUE 
    [[Q0],[Q1],[Q2],[Q3]]=Qn,
    QGtoL = [[Q0],[-Q1],[-Q2],[-Q3]],
    C= q2dcm(QGtoL), % Q should be a 4x1 vector
    Gn = [[0],[-1],[0]], % NOT 9.81 because we need unit vector  VHat*Vg= VHat*Vg/(|Vhat|*|Vg|)*cos(theta)
    MagN= [[1],[1],[0]], 
    % estimated vector of gravity and magnetic field
    % VgHat= unit(mat:'*'(C,Gn)),
    VgHat = unit(quatTransfomation(QGtoL,[[0],[-1],[0]])),
    [[VgHatX],[VgHatY],[VgHatZ]]=VgHat,

    % measured vector of gravity  and magnetic field
    Vg= unit([[-A]||A<-Acc]),

    Na= unit(cross_product(VgHat,Vg)),
    Domega= math:acos(dot(VgHat,Vg)),

    %error quaternion
    [[Qae2],[Qae3],[Qae4]]=consttimesVector(math:sin(Mua*Domega/2),Na),
    Qae1= math:cos(Mua*Domega/2),
    Qa = unit(q_product([[Qae1],[Qae2],[Qae3],[Qae4]],QGtoL)),
    Qainv= conjugateQuaternion(Qa),
    

    %Step 2:Correct the Estimated Direction of the Magnetic Field Using Magnetometer Readings
    [Vmx,Vmy,Vmz]=Mag,

    % Vmxz3= [Bx/math:sqrt(Bx*Bx+Bz*Bz),0,Bz/math:sqrt(Bx*Bx+Bz*Bz)],
    Vnorth= [[1],[0],[0]],
    VnorthL= quatTransfomation(Qa,Vnorth),
    Vmxyz= quatTransfomation(Qainv,[[Vmx],[Vmy],[Vmz]]),
    [[Bx],_,[Bz]]= Vmxyz,
    Vmxz= [[Bx/math:sqrt(Bx*Bx+Bz*Bz)],[0],[Bz/math:sqrt(Bx*Bx+Bz*Bz)]],

    VmxzL= quatTransfomation(Qa,Vmxz),
    Domegam= math:acos(dot(VnorthL,VmxzL)),
    [[Qme2],[Qme3],[Qme4]]=consttimesVector(math:sin(Domegam/2),cross_product(VnorthL,VmxzL)),
    Qme1= math:cos(Domegam/2),
    Qme = [[Qme1],[Qme2],[Qme3],[Qme4]],
    Qm=q_product(Qme,Qa),
    unit(conjugateQuaternion(Qm)).
%% An extended kalman filter without control input
ekf_predict({X0, P0}, {F, Jf}, Q) ->
    % Prediction
    Xp = F(X0),
    Jfx = Jf(X0),
    Pp = mat:eval([Jfx, '*', P0, '*´', Jfx, '+', Q]),
    {Xp,Pp}.
ekf_update({Xp,Pp},{Hxp,Jhx},R,Z)->
    % Update
    S = mat:eval([Jhx, '*', Pp, '*´', Jhx, '+', R]),
    Sinv = mat:inv(S),
    K = mat:eval([Pp, '*´', Jhx, '*', Sinv]),
    Y = mat:'-'(Z, Hxp),
    X1 = mat:eval([K, '*', Y, '+', Xp]),
    P1 = mat:'-'(Pp, mat:eval([K, '*', Jhx, '*', Pp])),
    {X1, P1}.
conjugateQuaternion([[Q0],[Q1],[Q2],[Q3]]) ->
    [[Q0],[-Q1],[-Q2],[-Q3]].
fromQuaternionTo3D([[_],[X],[Y],[Z]]) ->
    [[X],[Y],[Z]].
noNoiseUpdate(Xpos,Ppos,MeasurementPos)->
    [[X],[Y],[Z]]=MeasurementPos,
    [[_],[Vx],[Ax],[_],[Vy],[Ay],[_],[Vz],[Az]]=Xpos,
    {[[X],[Vx],[Ax],[Y],[Vy],[Ay],[Z],[Vz],[Az]],Ppos}. % return the updated position
create_list_with_one_at(Index, Length) ->
    create_list_with_one_at(Index, Length, 0).

create_list_with_one_at(_, Length, Length) ->
    []; % Base case: end of the list
create_list_with_one_at(Index, Length, Index) ->
    [1 | create_list_with_one_at(Index, Length, Index + 1)]; % Insert 1 at the specified index
create_list_with_one_at(Index, Length, CurrIndex) ->
    [0 | create_list_with_one_at(Index, Length, CurrIndex + 1)].
transpose_vector(Vector) ->
    [[Element] || Element <- Vector].
normalize_quat_state(State) ->
    State_dict = state_to_state_dict(State),
    Head_quat = [[maps:get(hq0, State_dict)], [maps:get(hq1, State_dict)], [maps:get(hq2, State_dict)], [maps:get(hq3, State_dict)]],
    Body_quat = [[maps:get(bq0, State_dict)], [maps:get(bq1, State_dict)], [maps:get(bq2, State_dict)], [maps:get(bq3, State_dict)]],
    Arm_quat = [[maps:get(aq0, State_dict)], [maps:get(aq1, State_dict)], [maps:get(aq2, State_dict)], [maps:get(aq3, State_dict)]],
    Forearm_quat = [[maps:get(fq0, State_dict)], [maps:get(fq1, State_dict)], [maps:get(fq2, State_dict)], [maps:get(fq3, State_dict)]],

    Norm_head_quat=unit(Head_quat),
    Norm_body_quat=unit(Body_quat),
    Norm_arm_quat=unit(Arm_quat),
    Norm_forearm_quat=unit(Forearm_quat),
    

    New_state = Norm_head_quat++[[maps:get(hpx, State_dict)], [maps:get(hpy, State_dict)], [maps:get(hpz, State_dict)]]++[[maps:get(h_vx, State_dict)], [maps:get(h_vy, State_dict)], [maps:get(h_vz, State_dict)]]++[[maps:get(h_ax, State_dict)], [maps:get(h_ay, State_dict)], [maps:get(h_az, State_dict)]]++[[maps:get(h_gx, State_dict)], [maps:get(h_gy, State_dict)], [maps:get(h_gz, State_dict)]]++
                Norm_body_quat++[[maps:get(bpx, State_dict)], [maps:get(bpy, State_dict)], [maps:get(bpz, State_dict)]]++[[maps:get(gBpx, State_dict)], [maps:get(gBpy, State_dict)], [maps:get(gBpz, State_dict)]]++
                Norm_arm_quat++[[maps:get(apx, State_dict)], [maps:get(apy, State_dict)], [maps:get(apz, State_dict)]]++[[maps:get(gApx, State_dict)], [maps:get(gApy, State_dict)], [maps:get(gApz, State_dict)]]++
                Norm_forearm_quat++[[maps:get(fpx, State_dict)], [maps:get(fpy, State_dict)], [maps:get(fpz, State_dict)]]++[[maps:get(gFpx, State_dict)], [maps:get(gFpy, State_dict)], [maps:get(gFpz, State_dict)]]++[[maps:get(time, State_dict)]],
    New_state.
% Sets the Nth element of a list to a new value
% N is 1-based index
set_element(N, List, Value) ->
    {Left, [_|Right]} = lists:split(N - 1, List),
    Left ++ [Value | Right].
