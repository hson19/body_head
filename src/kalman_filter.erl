-module(kalman_filter).

-behaviour(hera_measure).

-export([calibrate/1]).
-export([init/1, measure/1]).

-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate({MBx,MBy,MBz}) ->
    _ = io:get_line("Place the pmod_nav at 0° then press enter"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 100),
    [Mx,My,Mz] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    R0 = ahrs([Ax,Ay,-Az], [-(Mx-MBx),My-MBy,-(Mz-MBz)]),
    mat:tr(R0).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(R0) ->
    Spec = #{
        name => ?MODULE,
        iter => 1000,
        timeout => 10
    },
    T0 = hera:timestamp(),
    Xpos = mat:zeros(9, 1),
    Ppos = mat:eye(9),
    Xor = [[1],[0],[0],[0]],
    Por = mat:diag([10,10,10,10]),
    State = {T0, Xpos, Ppos, Xor, Por, R0},
    {ok, State, Spec}.


measure({T0, Xpos, Ppos, Xor, Por, R0}) ->
    DataNav = hera_data:get(nav3, body_head@body),
    DataHead = hera_data:get(nav3, body_head@head),
    io:format("found body_head@nav1 ~p~n", [DataNav] ),
    io:format("found body_head@head ~p~n", [DataHead] ),
    T1 = hera:timestamp(),
    Nav = [Data || {_,_,Ts,Data} <- DataNav, T0 < Ts, T1-Ts < 500],
    if
        length(Nav) == 0 -> % no measure
            {undefined, {T0, Xpos, Ppos, Xor, Por, R0}};
        true ->
            Orientation = q2dcm(Xor),
            {_, AccLin, _, _} = process_nav(Nav, Orientation),
            Dtpos = (T1-T0)/1000,

            Fpos = [
                [1,Dtpos,(Dtpos*Dtpos)/2,0,0,0,0,0,0], % X 
                [0,1,Dtpos,0,0,0,0,0,0], % V_X
                [0,0,1,0,0,0,0,0,0], % Acc_X
                [0,0,0,1,Dtpos,(Dtpos*Dtpos)/2,0,0,0], % Y
                [0,0,0,0,1,Dtpos,0,0,0], % V_Y
                [0,0,0,0,0,1,0,0,0], % Acc_Y
                [0,0,0,0,0,0,1,Dtpos,(Dtpos*Dtpos)/2], % Z
                [0,0,0,0,0,0,0,1,Dtpos], % V_Z
                [0,0,0,0,0,0,0,0,1] % Acc_Z
            ],
            Qpos = mat:diag([?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL]),
            Hpos =  [
                [1,0,0,0,0,0,0,0,0 ],
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,0,0,0,1,0,0], 
                [0,0,1,0,0,0,0,0,0],
                [0,0,0,0,0,1,0,0,0],
                [0,0,0,0,0,0,0,0,1] ],
            Zpos =  [[X] || X <- [1]] ++
                    [[Y] || Y <- [1]] ++
                    [[Z] || Z <- [1]] ++
                    [[Ax] || [Ax,_,_] <- AccLin] ++
                    [[Ay] || [_,Ay,_] <- AccLin] ++
                    [[Az] || [_,_,Az] <- AccLin],
            Rpos = mat:diag([?VAR_S,?VAR_S,?VAR_S,?VAR_AZ,?VAR_AZ,?VAR_AZ]),
            
            {Xpos0, Ppos0} = kalman:kf_predict({Xpos,Ppos}, Fpos, Qpos),
            {Xpos1,Ppos1} =  kalman:kf_update({Xpos0, Ppos0}, Hpos, Rpos, Zpos),
            Valor = unit([X || [X] <- Xor]),
            XorNorm = [[X] || X <- Valor],

            Valpos = lists:append(Xpos1),
            %print mesure
            io:format("Xpos: ~p~n", [Valpos]),
            {ok, Valpos ++ Valor, {T1, Xpos1, Ppos1, XorNorm, Por, R0}}
    end.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



process_nav([],_) -> 
    {[],[],[],[]};
process_nav([Nav], R) ->
    {Acc, Next} = lists:split(3, Nav),
    {Gyro, Mag} = lists:split(3, Next),
    Accrot = mat:'*'([Acc], mat:tr(R)),
    RotAcc = mat:'-'(Accrot, [[0,0,-9.81]]),
    {Acc, RotAcc, Gyro, Mag}. 


unit(V) ->
    Norm = math:sqrt(lists:sum([X*X || X <- V])),
    [X/Norm || X <- V].


scale(List, Factor) ->
    [X*Factor || X <- List].


calibrate(Comp, Registers, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].


ahrs(Acc, Mag) ->
    Down = unit([-A || A <- Acc]),
    East = unit(cross_product(Down, unit(Mag))),
    North = unit(cross_product(East, Down)),
    mat:tr([North, East, Down]).


cross_product([U1,U2,U3], [V1,V2,V3]) -> 
    [U2*V3-U3*V2, U3*V1-U1*V3, U1*V2-U2*V1].


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
    scale(V, (0.25/Q1)).


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