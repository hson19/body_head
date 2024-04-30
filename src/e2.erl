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
    R0 = ahrs([Ax,Ay,Az], [(Mx-MBx),My-MBy,(Mz-MBz)]), % why minus? ?? TODO
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
    BodyPos = [
        [0],[0],[0], % Position Body
        [0],[0],[0], % V BOdy
        [0],[0],[0]], % Acc Body
    BodyPpos = mat:eye(9),
    BodyOr= dcm2quat(R0),

    BodyPor = mat:diag([10,10,10,10]),
    BodyXpos = {T0,BodyPos, BodyPpos, BodyOr, BodyPor, R0},

    HeadPos=[[0],[0],[0.30], % Position Head
        [0],[0],[0], % V Head
        [0],[0],[0]], % Acc Head
    HeadPpos = mat:eye(9),
    HeadOr= dcm2quat(R0),
    io:format("HeadOr ~p~n",[HeadOr]),
    HeadPor = mat:diag([10,10,10,10]),
    HeadXpos = {T0,HeadPos, HeadPpos, HeadOr, HeadPor, R0},

    LeftArmPos=[[0.15],[0.15],[0.15], % Position LeftArm
        [0],[0],[0], % V LeftArm
        [0],[0],[0]], % Acc LeftArm
    LeftArmPpos = mat:eye(9),
    LeftArmOr= dcm2quat(R0),
    LeftArmPor = mat:diag([10,10,10,10]),
    LeftArmXpos = {T0,LeftArmPos, LeftArmPpos, LeftArmOr, LeftArmPor, R0},

    RightArmPos=[[-0.15],[0.15],[0.15], % Position RightArm
        [0],[0],[0], % V RightArm
        [0],[0],[0]], % Acc RightArm
    RightArmPpos = mat:eye(9),
    RightArmOr= dcm2quat(R0),
    RightArmPor = mat:diag([10,10,10,10]),
    RightArmXpos = {T0,RightArmPos, RightArmPpos, RightArmOr, RightArmPor, R0},

    State = {HeadXpos,BodyXpos,LeftArmXpos,RightArmXpos},

    {ok, State, Spec}.


measure({HeadXpos,BodyXpos,LeftArmXpos,RightArmXpos}) ->
    {T0Head,HeadPos,HeadPpos,HeadOr,HeadPor,HeadR0}=HeadXpos, 
    {T0Body,BodyPos,BodyPpos,BodyOr,BodyPor,BodyR0}=BodyXpos,
    {T0LeftArm,LeftArmPos,LeftArmPpos,LeftArmOr,LeftArmPor,LeftArmR0}=LeftArmXpos,
    {T0RightArm,RightArmPos,RightArmPpos,RightArmOr,RightArmPor,RightArmR0}=RightArmXpos,

    DataBody = hera_data:get(nav3, body_head@body),
    DataHead = hera_data:get(nav3, body_head@head),
    DataLeftArm = hera_data:get(nav3, body_head@left_arm),
    DataRightArm = hera_data:get(nav3, body_head@right_arm),

    T1 = hera:timestamp(),
    NavBody = [Data || {_,_,Ts,Data} <- DataBody, T0Body < Ts, T1-Ts < 500],
    NavHead = [Data || {_,_,Ts,Data} <- DataHead, T0Head < Ts, T1-Ts < 500],
    NavLeftArm = [Data || {_,_,Ts,Data} <- DataLeftArm, T0LeftArm < Ts, T1-Ts < 500],
    NavRightArm = [Data || {_,_,Ts,Data} <- DataRightArm, T0RightArm < Ts, T1-Ts < 500],
    if
        length(NavHead) == 0 -> % no measure
            UpdatedHeadXpos=HeadXpos,
            if 
                ?Debug_info ->HeadValpos = lists:append([HeadOr,HeadOr,HeadOr,HeadPos]);% DEBUG INFO
                true -> HeadValpos = lists:append([HeadOr,HeadPos])
            end;
        true ->
            {ok,HeadValpos,UpdatedHeadXpos}=sensorUpdate(NavHead,T1, HeadPos, HeadPpos, HeadOr, HeadPor, HeadR0,(T1-T0Head)/1000)
    end,
    if
        length(NavBody) == 0 -> % no measure
            UpdatedBodyXpos=BodyXpos,
            if 
                ?Debug_info ->BodyValpos = lists:append([BodyOr,BodyOr,BodyOr,BodyPos]);% DEBUG INFO
                true -> BodyValpos = lists:append([BodyOr,BodyPos])
            end;
    
        true ->
            io:format("Datat from NavBody"),
            {ok,BodyValpos,UpdatedBodyXpos}=sensorUpdate(NavBody,T1, BodyPos, BodyPpos, BodyOr, BodyPor, BodyR0,(T1-T0Body)/1000)
    end,
    if
        length(NavLeftArm) == 0 -> % no measure
            UpdatedLeftArmXpos=LeftArmXpos,
            if 
                ?Debug_info ->LeftArmValpos = lists:append([LeftArmOr,LeftArmOr,LeftArmOr,LeftArmPos]);% DEBUG INFO
                true -> LeftArmValpos = lists:append([LeftArmOr,LeftArmPos])
            end;
        true ->
            {ok,LeftArmValpos,UpdatedLeftArmXpos}=sensorUpdate(NavLeftArm,T1, LeftArmPos, LeftArmPpos, LeftArmOr, LeftArmPor, LeftArmR0,(T1-T0LeftArm)/1000)
    end,
    if 
        length(NavRightArm) == 0 -> % no measure
            UpdatedRightArmXpos=RightArmXpos,
            if 
                ?Debug_info ->RightArmValpos = lists:append([RightArmOr,RightArmOr,RightArmOr,RightArmPos]);% DEBUG INFO
                true -> RightArmValpos = lists:append([RightArmOr,RightArmPos])
            end;
        true ->
            {ok,RightArmValpos,UpdatedRightArmXpos}=sensorUpdate(NavRightArm,T1, RightArmPos, RightArmPpos, RightArmOr, RightArmPor, RightArmR0,(T1-T0RightArm)/1000)
    end,
    Valpos = lists:append([[T1],HeadValpos,BodyValpos,LeftArmValpos,RightArmValpos]),
    if length(NavBody)== 0 ->
        {undefined,{UpdatedHeadXpos,UpdatedBodyXpos,UpdatedLeftArmXpos,UpdatedRightArmXpos}};
        true-> {ok, Valpos, {UpdatedHeadXpos,UpdatedBodyXpos,UpdatedLeftArmXpos,UpdatedRightArmXpos}}
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

kalmanQuaternion(Acc,Mag,Gyro,Dt,BodyXor,P0,R0,Quat) ->
    [Wx,Wy,Wz] = Gyro,

    % Omega = [
    %     [0,-Wx,-Wy,-Wz],
    %     [Wx,0,Wz,-Wy],
    %     [Wy,-Wz,0,Wx],
    %     [Wz,Wy,-Wx,0]
    % ],
    Omega = [
        [0,Wx,Wy,Wz],
        [-Wx,0,-Wz,Wy],
        [-Wy,Wz,0,-Wx],
        [-Wz,-Wy,Wx,0]
    ],    
    F = mat:'+'(mat:eye(4), mat:'*'(0.5*Dt, Omega)),
    Q = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
    H = mat:eye(4),
    R = mat:diag([?VAR_R,?VAR_R,?VAR_R,?VAR_R]),

    {Xorp, Porp} = kalman:kf_predict({BodyXor,P0}, F, Q),


    {Xor1, Por1} = case qdot(Quat, Xorp) > 0 of
        true ->
            Qc= Quat,
            kalman:kf_update({Xorp, Porp}, H, R, Quat);
        false ->
            Qc= mat:'*'(-1,Quat),
            kalman:kf_update({mat:'*'(-1,Xorp), Porp}, H, R, Quat)            
        end,
        

    UnitXorp = unit(Xorp),

    % UnitXor1 = unit(Xor1),
    UnitXor1 = unit(Xor1),
    {ok, UnitXor1,Por1,UnitXorp,Qc}.

kalmanPositionPredict(Acc,Q,Dtpos,Pos,Ppos) ->
    Fpos = [
        [1,Dtpos,(Dtpos*Dtpos)/2,0,0,0,0,0,0], % North
        [0,1,Dtpos,0,0,0,0,0,0], % V_North
        [0,0,1,0,0,0,0,0,0], % Acc_North
        [0,0,0,1,Dtpos,(Dtpos*Dtpos)/2,0,0,0], % UP
        [0,0,0,0,1,Dtpos,0,0,0], % V_UP
        [0,0,0,0,0,1,0,0,0], % Acc_UP
        [0,0,0,0,0,0,1,Dtpos,(Dtpos*Dtpos)/2], % EAST
        [0,0,0,0,0,0,0,1,Dtpos], % V_EAST
        [0,0,0,0,0,0,0,0,1] % Acc_EAST
    ],
    Qpos = mat:diag([?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL]),
    [[X],[Vx],[_],[Y],[Vy],[_],[Z],[Vz],[_]] = Pos,
    [A2,A3,A4]=Acc,
    % io:format("read acc ~p~n",[Acc]),
    AccTrans = quatTransfomation(Q,[[0],[A2],[A3],[A4]]),
    [[_],[AccNorth],[AccUp],[AccEast]] = AccTrans,
    % io:format("transformed acc ~p~n",[AccTrans]),
    NewPos= [[X],[Vx],[AccNorth],[Y],[Vy],[AccUp-9.81],[Z],[Vz],[AccEast]],

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
    QGtoL = [[Q0],[-Q1],[-Q2],[-Q3]],
    C= q2dcm(QGtoL), % Q should be a 4x1 vector
    Gn = [[0],[-1],[0]], % NOT 9.81 because we need unit vector  VHat*Vg= VHat*Vg/(|Vhat|*|Vg|)*cos(theta)
    MagN= [[1],[1],[0]], 
    % estimated vector of gravity and magnetic field
    % VgHat= unit(mat:'*'(C,Gn)),
    VgHat = unit(quatTransfomation(QGtoL,[[0],[0],[-1],[0]])),
    [[_],[VgHatX],[VgHatY],[VgHatZ]]=VgHat,

    % measured vector of gravity  and magnetic field
    Vg= unit([-A||A<-Acc]),

    Na= unit(cross_product([VgHatX,VgHatY,VgHatZ],Vg)),
    Domega= math:acos(dot([VgHatX,VgHatY,VgHatZ],Vg)),

    %error quaternion
    [Qae2,Qae3,Qae4]=consttimesVector(math:sin(Mua*Domega/2),Na),
    Qae1= math:cos(Mua*Domega/2),
    Qa = unit(q_product([[Qae1],[Qae2],[Qae3],[Qae4]],QGtoL)),
    % Qa.
    [[Qa1],[Qa2],[Qa3],[Qa4]]=Qa,
    Qainv= [[Qa1],[-Qa2],[-Qa3],[-Qa4]],
    % Qainv.
    

    % % %Step 2:Correct the Estimated Direction of the Magnetic Field Using Magnetometer Readings
    [Vmx,Vmy,Vmz]=Mag,

    % Vmxz3= [Bx/math:sqrt(Bx*Bx+Bz*Bz),0,Bz/math:sqrt(Bx*Bx+Bz*Bz)],
    Vnorth= [1,0,0],
    VnorthL= quatTransfomation(Qa,[[0],[1],[0],[0]]),
    Vmxyz= quatTransfomation(Qainv,[[0],[Vmx],[Vmy],[Vmz]]),
    [_,[Bx],_,[Bz]]= Vmxyz,
    Vmxz= [[0],[Bx/math:sqrt(Bx*Bx+Bz*Bz)],[0],[Bz/math:sqrt(Bx*Bx+Bz*Bz)]],

    VmxzL= quatTransfomation(Qa,Vmxz),
    [[_],[VmxzLx],[VmxzLy],[VmxzLz]]=VmxzL,
    [[_],[VnorthLx],[VnorthLy],[VnorthLz]]=VnorthL,
    Domegam= math:acos(dot([VnorthLx,VnorthLy,VnorthLz],[VmxzLx,VmxzLy,VmxzLz])),
    [Qme2,Qme3,Qme4]=consttimesVector(math:sin(Domegam/2),cross_product([VnorthLx,VnorthLy,VnorthLz],[VmxzLx,VmxzLy,VmxzLz])),
    Qme1= math:cos(Domegam/2),
    Qme = [[Qme1],[Qme2],[Qme3],[Qme4]],
    Qm=q_product(Qme,Qa),
    unit(conjugateQuaternion(Qm)).
    % [Mx,My,Mz]= Mag,
    % Norm= math:sqrt(Mx*Mx+My*My+Mz*Mz),
    % HCONST=10, %TODO FIND VALUE
    % if (Norm - HCONST) > 0.1 -> 
    %     q_product(Qa,Qn);
    % true -> 
    %     q_product(Qm,Qn)
    % end.
    % THE magnecic field intensity is stable
    % [[W],[X],[Y],[Z]]=q_product(Qm,Qn),
    % unit([[W],[X],[Y],[Z]]). % return the quaternion

sensorUpdate(Data,T1, BodyXpos, BodyPpos, BodyXor, BodyPor, BodyR0,Dt) ->
    {Acc,RotAcc,Gyro, Mag} = process_nav(Data, BodyR0),
    
    Qc = observationModel(BodyXor,Acc,Mag,BodyR0),
    io:format("Qc ~p~n",[Qc]),
    io:format("Gyro ~p~n",[Gyro]),
    {_, Xor1,Por1,Xorp,UpdatedQc}=kalmanQuaternion(Acc, Mag, Gyro, Dt,BodyXor,BodyPor,BodyR0,Qc), %Acc,Mag,Gyro,Dt,BodyXor,P0,R0
    io:format("UpdatedQc ~p~n",[UpdatedQc]),
    io:format("Xor1 ~p~n",[Xor1]),
    io:format("Xorp ~p~n",[Xorp]),
    
    {BodyXpos0, BodyPpos0} = kalmanPositionPredict(Acc,BodyXor, Dt,BodyXpos,BodyPpos),
    if 
    ?Debug_info ->Valpos = lists:append([Xor1,Xorp,UpdatedQc,BodyXpos0]);% DEBUG INFO
    true -> Valpos = lists:append([Xor1,BodyXpos0])
    end,

    % io:format("BodyXpos0 ~p~n", [BodyXpos0]),
    {ok,Valpos, {T1,BodyXpos0,BodyPpos0,Xorp,Por1,BodyR0}}. %TODO chang Xorp to Xor1

conjugateQuaternion([[Q0],[Q1],[Q2],[Q3]]) ->
    [[Q0],[-Q1],[-Q2],[-Q3]].



