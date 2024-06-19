-module(e4).

-behaviour(hera_measure).

-export([calibrate/1]).
-export([init/1, measure/1]).
-export([observationModel/3]).

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
        timeout => 10
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
    NavBody = [Data || {_,_,Ts,Data} <- DataBody, T0 < Ts, T1-Ts < 500],
    NavHead = [Data || {_,_,Ts,Data} <- DataHead, T0 < Ts, T1-Ts < 500],
    NavArm = [Data || {_,_,Ts,Data} <- DataArm, T0 < Ts, T1-Ts < 500],
    NavForearm = [Data || {_,_,Ts,Data} <- Dataforearm, T0 < Ts, T1-Ts < 500],
    {_,Card}=getNewestSensorData(NavHead,NavBody,NavArm,NavForearm),
    State_dict_dt=maps:put(time, (T1-maps:get(time,State_dict))/1000, State_dict), % change the time to dt
    State_prediction=state_dict_to_state(State_dict_dt),
    
    if Card == head ->    
        % ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z) 
        {Acc, Gyro, Mag}=process_nav(NavHead),
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
        Z=Qc++[[Ax],[Ay],[Az]]++[[Gx],[Gy],[Gz]],
        Q=mat:eye(46),
        R= mat:eye(10),
        {NewState,P1}=kalman:ekf({State_prediction,P0}, {fun state_prediction/1,fun jacobian_state_prediction/1},{fun head_state_measurement/1,fun jacobian_state_measurement/1}, Q,R,Z),
        New_state1=NewState ++ [[T1]],
        Normalized_new_state=normalize_quat_state(New_state1),
        Valpos=Normalized_new_state, 
        {ok,Valpos,{Normalized_new_state,P1}};
    Card == body ->
        {Acc, Gyro, Mag}=process_nav(NavBody),
        Qbody=[maps:get(bq0, State_dict_dt), maps:get(bq1, State_dict_dt), maps:get(bq2, State_dict_dt), maps:get(bq3, State_dict_dt)],
        Z=lists:flatten([observationModel(Qbody,Acc,Mag),Acc,Gyro]),
        Q=mat:eye(46),
        R= mat:eye(9),
        {NewState,P1}=kalman:ekf({State,P0}, {fun state_prediction/1,fun jacobian_state_prediction/1},{fun head_state_measurement/1,fun jacobian_state_measurement/1}, Q,R,Z),
        New_state1=NewState ++ [[T1]],
        Normalized_new_state=normalize_quat_state(New_state1),
        Valpos=Normalized_new_state, 
        {ok,Valpos,{NewState,P1}};
    Card == arm ->
        {Acc, Gyro, Mag}=process_nav(NavArm),
        Qarm=[maps:get(aq0, State_dict_dt), maps:get(aq1, State_dict_dt), maps:get(aq2, State_dict_dt), maps:get(aq3, State_dict_dt)],
        Z=lists:flatten([observationModel(Qarm,Acc,Mag),Acc,Gyro]),
        Q=mat:eye(46),
        R= mat:eye(9),
        {NewState,P1}=kalman:ekf({State,P0}, {fun state_prediction/1,fun jacobian_state_prediction/1},{fun head_state_measurement/1,fun jacobian_state_measurement/1}, Q,R,Z),
        New_state1=NewState ++ [[T1]],
        Normalized_new_state=normalize_quat_state(New_state1),
        Valpos=Normalized_new_state,     
        {ok,Valpos,{NewState,P1}};
    Card == forearm ->
        {Acc, Gyro, Mag}=process_nav(NavForearm),
        Qforearm=[maps:get(fq0, State_dict_dt), maps:get(fq1, State_dict_dt), maps:get(fq2, State_dict_dt), maps:get(fq3, State_dict_dt)],
        Z=lists:flatten([observationModel(Qforearm,Acc,Mag),Acc,Gyro]),
        Q=mat:eye(46),
        R= mat:eye(9),
        {NewState,P1}=kalman:ekf({State,P0}, {fun state_prediction/1,fun jacobian_state_prediction/1},{fun head_state_measurement/1,fun jacobian_state_measurement/1}, Q,R,Z),
        New_state1=NewState ++ [[T1]],
        Normalized_new_state=normalize_quat_state(New_state1),
        Valpos=Normalized_new_state,     
        {ok,Valpos,{NewState,P1}};
    true ->
        {undefined, {State,P0}}
    end.
    

            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize state_dict
getNewestSensorData([], [], [], []) ->
    io:format("All data lists are empty~n"),
    {[], none};
getNewestSensorData(Head_data, Body_data, Arm_data, Forearm_data) ->
    DataLists = [Head_data, Body_data, Arm_data, Forearm_data],
    % Ensure each list has at least one element and the first element is a number (T1)
    ValidDataLists = lists:filter(fun
        (Data) when is_list(Data), length(Data) > 0-> true;
        (_) -> false
    end, DataLists),

    case ValidDataLists of
        [] ->
            io:format("All data lists are empty or invalid~n"),
            {[], none};
        _ ->
            % Convert lists to tuples {T1, Data}
            Tuples = lists:map(fun([T1 | Rest]) -> {T1, Rest} end, ValidDataLists),
            
            % Find the tuple with the smallest T1
            MinTuple = lists:min(Tuples),
            
            % Match MinTuple to the corresponding data and return
            case MinTuple of
                {T1, _} when MinTuple == {hd(Head_data), tl(Head_data)} -> {Head_data, head};
                {T1, _} when MinTuple == {hd(Body_data), tl(Body_data)} -> {Body_data, body};
                {T1, _} when MinTuple == {hd(Arm_data), tl(Arm_data)} -> {Arm_data, arm};
                {T1, _} when MinTuple == {hd(Forearm_data), tl(Forearm_data)} -> {Forearm_data, forearm}
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
    {ok,New_arm_quat} = kalman_quaternion_predict(Arm_g, Dt, Arm_quat, mat:eye(4)),
    Shoulder_pos = mat:'+'(quatTransfomation(Head_quat,[[0],[-?HEADSHOULDERDISTANCE],[0]]),Head_pos),
    Shoulder_to_biceps = quatTransfomation(Arm_quat,[[0],[-?SHOULDERBICEPSDISTANCE],[0]]),
    New_arm_pos = mat:'+'(Shoulder_pos,Shoulder_to_biceps),
    % Forearm prediction
    {ok,New_forearm_quat} = kalman_quaternion_predict(Forearm_g, Dt, Forearm_quat, mat:eye(4)),
    Elbow_pos = mat:'+'(quatTransfomation(Arm_quat,[[0],[-?BICEPSELBOWDISTANCE],[0]]),Shoulder_pos),
    New_forearm_pos = mat:'+'(Elbow_pos,quatTransfomation(Forearm_quat,[[0],[-?ELBOWWRISTDISTANCE],[0]])),
    % Create a list of new values
    New_values = New_head_quat++New_head_pos++New_head_v++Head_a++Head_g++
                                New_body_quat++New_body_pos++Body_g++
                                New_arm_quat++New_arm_pos++Arm_g++
                                New_forearm_quat++New_forearm_pos++Forearm_g,
    New_values.


jacobian_state_prediction(_) ->
    mat:eye(46).
head_state_measurement(State)-> 
    H=jacobian_state_measurement(State),
    State_without_time=lists:sublist(State,46),
    Result=mat:'*'(H,State_without_time),
    Result.
jacobian_state_measurement(State) ->
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
process_nav([Nav]) ->
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




%Hierachical Model with the Head as the parent but enhance Head position using body position
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