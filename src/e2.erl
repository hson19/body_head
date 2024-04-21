-module(e2).

-behaviour(hera_measure).

-export([calibrate/1]).
-export([init/1, measure/1]).

-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).
-define(Debug_info, true).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate({MBx,MBy,MBz}) ->
    _ = io:get_line("Place the pmod_nav at 0° then press enter"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 100),
    [Mx,My,Mz] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    R0 = ahrs([Ax,Ay,Az], [-(Mx-MBx),My-MBy,-(Mz-MBz)]),
    io:format("Calibration matrice at calibration: ~p~n", [mat:tr(R0)]),
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
    BodyXpos = [
        [0],[0],[0], % Position Body
        [0],[0],[0], % V BOdy
        [0],[0],[0]], % Acc Body
    HeadXpos=[[0],[0],[0.30], % Position Head
        [0],[0],[0], % V Head
        [0],[0],[0]], % Acc Head
    
    Ppos = mat:eye(9),
    Q= dcm2quat(R0),
    io:format("calibration matrice at init: ~p~n", [R0]),
    [Q1,Q2,Q3,Q4] = Q,
    Xor = [[Q1],[-Q2],[-Q3],[-Q4]],
    % Xor = [[1],[0],[0],[0]], % quaternion representation of the orientation
    Por = mat:diag([10,10,10,10]),
    State = {T0, BodyXpos, Ppos, Xor, Por, R0},
    {ok, State, Spec}.


measure({T0, BodyXpos, BodyPpos, BodyXor, BodyPor, BodyR0}) ->
    DataBody = hera_data:get(nav3, body_head@body),
    DataHead = hera_data:get(nav3, body_head@head),
    T1 = hera:timestamp(),
    % NavBody = [Data || {_,_,Ts,Data} <- DataBody, T0 < Ts, T1-Ts < 500],
    NavHead = [Data || {_,_,Ts,Data} <- DataHead, T0 < Ts, T1-Ts < 500],
    %io:format("found body_head@head ~p~n", [NavHead] ),
    if
        length(NavHead) == 0 -> % no measure
            {undefined, {T0, BodyXpos, BodyPpos, BodyXor, BodyPor, BodyR0}};
        true ->
            {Acc,RotAcc,Gyro, Mag} = process_nav(NavHead, BodyR0),
            Dt = (T1-T0)/1000,
            Qc = observationModel(BodyXor,Acc,Mag,BodyR0),
            {_, Xor1,Por1,Xorp}=kalmanQuaternion(Acc, Mag, Gyro, Dt,BodyXor,BodyPor,BodyR0,Qc), %Acc,Mag,Gyro,Dt,BodyXor,P0,R0
            {BodyXpos0, BodyPpos0} = kalmanPositionPredict(Acc,BodyXor, Dt,BodyXpos,BodyPpos),
            if 
            ?Debug_info ->Valpos = lists:append([[T1],Xor1,Xorp,Qc,BodyXpos0]);% DEBUG INFO
            true -> Valpos = lists:append([[T1],Xor1,BodyXpos0])
            end,
            io:format("Xor ~p~n", [Xor1]),
            io:format("BodyXpos0 ~p~n", [BodyXpos0]),
            {ok,Valpos, {T1, BodyXpos0,BodyPpos0,Xorp,Por1,BodyR0}} %TODO chang Xorp to Xor1
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


qdot([[Q11], [Q12], [Q13], [Q14]], [[Q21], [Q22], [Q23], [Q24]]) ->
    Q11*Q21 + Q12*Q22 + Q13*Q23 + Q14*Q24.
dot([X1,X2,X3], [Y1,Y2,Y3]) ->
    X1*Y1 + X2*Y2 + X3*Y3.
consttimesVector(C, [[X],[Y],[Z]]) ->
    [[C*X], [C*Y], [C*Z]];
consttimesVector(C, [X,Y,Z]) ->
    [C*X, C*Y, C*Z].
process_nav([],_) -> 
    {[],[],[],[]};
process_nav([Nav], R) ->
    {Acc, Next} = lists:split(3, Nav),
    {Gyro, Mag} = lists:split(3, Next),
    Accrot = mat:'*'([Acc], mat:tr(R)),
    RotAcc = mat:'-'(Accrot, [[0,0,-9.81]]),
    {Acc, RotAcc, Gyro, Mag}. 

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

kalmanQuaternion(Acc,Mag,Gyro,Dt,BodyXor,P0,R0,Quat) ->
    [Wx,Wy,Wz] = Gyro,

    Omega = [
        [0,-Wx,-Wy,-Wz],
        [Wx,0,Wz,-Wy],
        [Wy,-Wz,0,Wx],
        [Wz,Wy,-Wx,0]
    ],
    F = mat:'+'(mat:eye(4), mat:'*'(0.5*Dt, Omega)),
    Q = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
    H = mat:eye(4),
    R = mat:diag([?VAR_R,?VAR_R,?VAR_R,?VAR_R]),

    {Xorp, Porp} = kalman:kf_predict({BodyXor,P0}, F, Q),
    {Xor1, Por1} = case qdot(Quat, Xorp) > 0 of
        true ->
            kalman:kf_update({Xorp, Porp}, H, R, Quat);
        false ->
            kalman:kf_update({mat:'*'(-1,Xorp), Porp}, H, R, Quat)
        end,
    % {Xor1,Por1} = {Xorp, Porp}, % gyro only
    io:format("Xor1 ~p~n",[Xor1]),
    io:format("Quat ~p~n",[Quat]),
    [[Qp1],[Qp2],[Qp3],[Qp4]]=Xorp,
    Normp=math:sqrt(Qp1*Qp1+Qp2*Qp2+Qp3*Qp3+Qp4*Qp4),
    UnitXorp = [[Qp1/Normp],[Qp2/Normp],[Qp3/Normp],[Qp4/Normp]],

    % UnitXor1 = unit(Xor1),
    [[Q1],[Q2],[Q3],[Q4]]=Xor1,
    Norm=math:sqrt(Q1*Q1+Q2*Q2+Q3*Q3+Q4*Q4),
    UnitXor1 = [[Q1/Norm],[Q2/Norm],[Q3/Norm],[Q4/Norm]],
    {ok, UnitXor1,Por1,UnitXorp}.

kalmanPositionPredict(Acc,Q,Dtpos,Pos,Ppos) ->
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
    [[X],[Vx],[_],[Y],[Vy],[_],[Z],[Vz],[_]] = Pos,
    [A2,A3,A4]=Acc,
    % io:format("read acc ~p~n",[Acc]),
    AccTrans = quatTransfomation(Q,[[0],[A2],[A3],[A4]]),
    % io:format("transformed acc ~p~n",[AccTrans]),
    [[_],[NewAx],[NewAy],[NewAz]] = AccTrans,
    NewPos= [[X],[Vx],[NewAx],[Y],[Vy],[NewAy],[Z],[Vz],[NewAz]],
    
    {Xpos0, Ppos0} = kalman:kf_predict({NewPos,Ppos}, Fpos, Qpos),
    {Xpos0, Ppos0}.

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
    Fp= q_product(Q,q_product(P,Qinv)),
    Fp.

observationModel(Qn,Acc,Mag,R0) ->
    % transform quaterion to matrice
    Mua = 1, %0.005, %TODO FIND VALUE 
    [[Q0],[Q1],[Q2],[Q3]]=Qn,
    Qinv = [[Q0],[-Q1],[-Q2],[-Q3]],
    C= q2dcm(Qinv), % Q should be a 4x1 vector
    Gn = [[0],[-1],[0]], % NOT 9.81 because we need unit vector  VHat*Vg= |VHat|*|Vg|*cos(theta)
    MagN= [[1],[1],[0]], 
    % estimated vector of gravity and magnetic field
    VgHat= unit(mat:'*'(C,Gn)),
    [[VgHatX],[VgHatY],[VgHatZ]]=VgHat,
    VmHat=unit(mat:'*'(C,MagN)), 
    % measured vector of gravity  and magnetic field
    Vg= unit(Acc),
    Vm=unit(Mag),
    Na= cross_product([VgHatX,VgHatY,VgHatZ],Vg),
    Domega= math:acos(dot([VgHatX,VgHatY,VgHatZ],Vg)),
    io:format("VgHat ~p~n",[VgHat]),
    io:format("Vg ~p~n",[Vg]),
    %error quaternion
    io:format("Na ~p~n",[Na]),
    io:format("Domega ~p~n",[Domega]),
    [Qae2,Qae3,Qae4]=consttimesVector(math:sin(Mua*Domega/2),Na),
    io:format("Qae ~p~n",[[Qae2,Qae3,Qae4]]),
    Qae1= math:cos(Mua*Domega/2),
    Qa = q_product([[Qae1],[Qae2],[Qae3],[Qae4]],Qn),
    Qa.
    

    % %Step 2:Correct the Estimated Direction of the Magnetic Field Using Magnetometer Readings
    % [Vmx,Vmy,Vmz]=Mag,
    % Vmxz= quatTransfomation(Qa,[[0],[Vmx],[Vmy],[Vmz]]),
    % [_,[Bx],_,[Bz]]= Vmxz,
    % Vmxz3= [math:sqrt(Bx*Bx+Bz*Bz),0,0],
    % Vnorth= [1,0,0],
    % Domegam= math:acos(dot(Vnorth,Vmxz3)),
    % [Qme2,Qme3,Qme4]=consttimesVector(math:sin(Domegam/2),cross_product(Vmxz3,Vnorth)),
    % Qme1= math:cos(Domegam/2),
    % Qme = [[Qme1],[Qme2],[Qme3],[Qme4]],
    % Qm=q_product(Qme,Qa),
    % % [Mx,My,Mz]= Mag,
    % % Norm= math:sqrt(Mx*Mx+My*My+Mz*Mz),
    % % HCONST=10, %TODO FIND VALUE
    % % if (Norm - HCONST) > 0.1 -> 
    % %     q_product(Qa,Qn);
    % % true -> 
    % %     q_product(Qm,Qn)
    % % end.
    % % THE magnecic field intensity is stable
    % [[Q1],[Q2],[Q3],[Q4]]=q_product(Qm,Qn),
    % Norm= math:sqrt(Q1*Q1+Q2*Q2+Q3*Q3+Q4*Q4),
    % [[Q1/Norm],[Q2/Norm],[Q3/Norm],[Q4/Norm]].




