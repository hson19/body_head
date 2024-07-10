-module(e2).

-behaviour(hera_measure).

-export([calibrate/1]).
-export([init/1, measure/1]).
-export([observationModel/4]).

-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).
-define(Debug_info, true).
-define(BodyHeadDistance, 0.30).
-define(HeadArmDistance, 0.25).
-define(ArmCenterBiceps, 0.15).
-define(BicepsElbow, 0.15).
-define(ElbowWrist, 0.15).
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
    HeadPos=[[0],[0],[0], % Position Head
        [0],[0],[0], % V Head
        [0],[0],[0]], % Acc Head
    HeadPpos = mat:eye(9),
    HeadOr= dcm2quat(R0),
    io:format("HeadOr ~p~n",[HeadOr]),
    HeadPor = mat:diag([10,10,10,10]),
    HeadXpos = {T0,HeadPos, HeadPpos, HeadOr, HeadPor, R0},

    
    BodyPosition = quatTransfomation(HeadOr,[[0],[-?BodyHeadDistance],[0],[0]]),
    [[_],[Bodyx],[Bodyy],[Bodyz]]=BodyPosition,
    BodyPos = [
        [Bodyx],[0],[0], % Position Body
        [Bodyy],[0],[0], % V BOdy
        [Bodyz],[0],[0]], % Acc Body
    BodyPpos = mat:eye(9),
    BodyOr= dcm2quat(R0),

    BodyPor = mat:diag([10,10,10,10]),
    BodyXpos = {T0,BodyPos, BodyPpos, BodyOr, BodyPor, R0},

    ArmPosition = quatTransfomation(HeadOr,[[0],[-?HeadArmDistance],[-?ArmCenterBiceps],[0]]),
    [[_],[Armx],[Army],[Armz]]=ArmPosition,
    ArmPos=[[Armx],[0],[0], % Position LeftArm
        [Army],[0],[0], % V LeftArm
        [Armz],[0],[0]], % Acc LeftArm
    ArmPpos = mat:eye(9),
    ArmOr= dcm2quat(R0),
    ArmPor = mat:diag([10,10,10,10]),
    ArmXpos = {T0,ArmPos, ArmPpos, ArmOr, ArmPor, R0},
    
    ForearmPosition= quatTransfomation(HeadOr,[[0],[-?HeadArmDistance],[-?ArmCenterBiceps-?BicepsElbow],[0]]),
    [[_],[Forearmx],[Forearmy],[Forearmz]]=ForearmPosition,
    ForearmPos=[[Forearmx],[0],[0], % Position Forearm
        [Forearmy],[0],[0], % V Forearm
        [Forearmz],[0],[0]], % Acc Forearm
    ForearmPpos = mat:eye(9),
    ForearmOr= dcm2quat(R0),
    ForearmPor = mat:diag([10,10,10,10]),
    ForearmXpos = {T0,ForearmPos, ForearmPpos, ForearmOr, ForearmPor, R0},

    State = {HeadXpos,BodyXpos,ArmXpos,ForearmXpos},

    {ok, State, Spec}.


measure({HeadXpos,BodyXpos,ArmXpos,ForearmXpos}) ->
    {T0Head,HeadPos,HeadPpos,HeadOr,HeadPor,HeadR0}=HeadXpos, 
    {T0Body,BodyPos,BodyPpos,BodyOr,BodyPor,BodyR0}=BodyXpos,
    {T0Arm,ArmPos,ArmPpos,ArmOr,ArmPor,ArmR0}=ArmXpos,
    {T0Forearm,ForearmPos,ForearmPpos,ForearmOr,ForearmPor,ForearmR0}=ForearmXpos,

    DataBody = hera_data:get(nav3, body_head@body),
    DataHead = hera_data:get(nav3, body_head@head),
    DataArm = hera_data:get(nav3, body_head@arm),
    Dataforearm = hera_data:get(nav3, body_head@forearm),

    T1 = hera:timestamp(),
    NavBody = [Data || {_,_,Ts,Data} <- DataBody, T0Body < Ts, T1-Ts < 500],
    NavHead = [Data || {_,_,Ts,Data} <- DataHead, T0Head < Ts, T1-Ts < 500],
    NavArm = [Data || {_,_,Ts,Data} <- DataArm, T0Arm < Ts, T1-Ts < 500],
    NavForearm = [Data || {_,_,Ts,Data} <- Dataforearm, T0Forearm < Ts, T1-Ts < 500],
    {AccHead,_,_,Mag}= process_nav(NavHead,HeadR0),
    if
        length(NavHead) == 0 -> % no measure
            UpdatedHeadXpos=HeadXpos,
            if 
                ?Debug_info ->HeadValpos = lists:append([HeadOr,HeadOr,HeadOr,HeadPos]);% DEBUG INFO
                true -> HeadValpos = lists:append([HeadOr,HeadPos])
            end;
        true ->
            {ok,HeadValpos,UpdatedHeadXpos}=sensorUpdate(NavHead,T1, HeadPos, HeadPpos, HeadOr, HeadPor, HeadR0,(T1-T0Head)/1000,head)
    end,
    if
        length(NavBody) == 0 -> % no measure
            UpdatedBodyXpos=BodyXpos,
            if 
                ?Debug_info ->BodyValpos = lists:append([BodyOr,BodyOr,BodyOr,BodyPos]);% DEBUG INFO
                true -> BodyValpos = lists:append([BodyOr,BodyPos])
            end;
    
        true ->
            io:format("Data from NavBody"),
            {ok,BodyValpos,UpdatedBodyXpos}=sensorUpdate(NavBody,T1, BodyPos, BodyPpos, BodyOr, BodyPor, BodyR0,(T1-T0Body)/1000,body)
    end,
    if
        length(NavArm) == 0 -> % no measure
            UpdatedArmXpos=ArmXpos,
            if 
                ?Debug_info ->ArmValpos = lists:append([ArmOr,ArmOr,ArmOr,ArmPos]);% DEBUG INFO
                true -> ArmValpos = lists:append([ArmOr,ArmPos])
            end;
        true ->
            {ok,ArmValpos,UpdatedArmXpos}=sensorUpdate(NavArm,T1, ArmPos, ArmPpos, ArmOr, ArmPor, ArmR0,(T1-T0Arm)/1000,arm)
    end,
    if 
        length(NavForearm) == 0 -> % no measure
            UpdatedForearmXpos=ForearmXpos,
            if 
                ?Debug_info ->ForearmValpos = lists:append([ForearmOr,ForearmOr,ForearmOr,ForearmPos]);% DEBUG INFO
                true -> ForearmValpos = lists:append([ForearmOr,ForearmPos])
            end;
        true ->
            {ok,ForearmValpos,UpdatedForearmXpos}=sensorUpdate(NavForearm,T1, ForearmPos, ForearmPpos, ForearmOr, ForearmPor, ForearmR0,(T1-T0Forearm)/1000,arm)
    end,
    
   % start applying the constraint Hierachical Model 
    if ((length(NavHead) > 0) orelse (length(NavBody) > 0) ) ->
        {ConstraintHeadXpos,ConstraintBodyXpos,ConstraintHeadValPos,ConstraintBodyValPos}=bodyHeadConstraint(UpdatedBodyXpos,UpdatedHeadXpos,HeadValpos,BodyValpos),
        {ConstraintArmXpos,ConstraintArmValpos}=headArmConstraint(ConstraintHeadXpos,UpdatedArmXpos,right),
        {ConstraintForearmXpos,ConstraintForearmValpos}=armForearmConstraint(ConstraintArmXpos,UpdatedForearmXpos),
        Valpos = lists:append([[T1],ConstraintHeadValPos,ConstraintBodyValPos,ConstraintArmValpos,ConstraintForearmValpos,AccHead,Mag]),
        io:format("TimeHead ~p~n",[hera:timestamp()-T1]),
        {ok,Valpos, {ConstraintHeadXpos,ConstraintBodyXpos,ConstraintArmXpos,ConstraintForearmXpos}};
    length(NavArm) > 0 ->
        {ConstraintArmXpos,ConstraintArmValpos}=headArmConstraint(UpdatedHeadXpos,UpdatedArmXpos,right),
        {ConstraintForearmXpos,ConstraintForearmValpos}=armForearmConstraint(ConstraintArmXpos,UpdatedForearmXpos),
        Valpos = lists:append([[T1],HeadValpos,BodyValpos,ConstraintArmValpos,ConstraintForearmValpos,AccHead,Mag]),
        io:format("TimeArm ~p~n",[hera:timestamp()-T1]),
        {ok,Valpos, {UpdatedHeadXpos, UpdatedBodyXpos, ConstraintArmXpos,ConstraintForearmXpos}};
    length(NavForearm) > 0 ->
        {ConstraintForearmXpos,ConstraintForearmValpos}=armForearmConstraint(UpdatedArmXpos,UpdatedForearmXpos),
        Valpos = lists:append([[T1],HeadValpos,BodyValpos,ArmValpos,ConstraintForearmValpos,AccHead,Mag]),
        io:format("TimeForearm ~p~n",[hera:timestamp()-T1]),
        {ok,Valpos, {UpdatedHeadXpos, UpdatedBodyXpos, UpdatedArmXpos,ConstraintForearmXpos}};
    true -> {undefined, {UpdatedHeadXpos, UpdatedBodyXpos, UpdatedArmXpos, UpdatedForearmXpos}}
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
    Omega =[ 
        [0,-Wx,-Wy,-Wz],
        [Wx,0,-Wz,Wy],
        [Wy,Wz,0,-Wx],
        [Wz,-Wy,Wx,0]
    ],
    F = mat:'+'(mat:eye(4), mat:'*'(0.5*Dt, Omega)),
    Q = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
    H = mat:eye(4),
    R = mat:diag([?VAR_R,?VAR_R,?VAR_R,?VAR_R]),
    Quaternion= conjugateQuaternion(BodyXor),
    {Xorp, Porp} = kalman:kf_predict({Quaternion,P0}, F, Q),

    ConjQuat= conjugateQuaternion(Quat),
    {Xor1, Por1} = case qdot(ConjQuat, Xorp) > 0 of
        true ->
            Qc= Quat,
            kalman:kf_update({Xorp, Porp}, H, R, ConjQuat);
        false ->
            Qc= conjugateQuaternion(mat:'*'(-1,ConjQuat)),
            kalman:kf_update({Xorp, Porp}, H, R, Qc)            
        end,
        

    UnitXorp = unit(conjugateQuaternion(Xorp)),

    UnitXor1 = unit(conjugateQuaternion(Xor1)),
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
    AccTrans = quatTransfomation(Q,[[0],[A2],[A3],[A4]]),
    [[_],[AccNorth],[AccUp],[AccEast]] = AccTrans,
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
    [[Qa1],[Qa2],[Qa3],[Qa4]]=Qa,
    Qainv= [[Qa1],[-Qa2],[-Qa3],[-Qa4]],
    

    %Step 2:Correct the Estimated Direction of the Magnetic Field Using Magnetometer Readings
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


sensorUpdate(Data,T1, BodyXpos, BodyPpos, BodyXor, BodyPor, BodyR0,Dt,BodyPart) ->
    {Acc,RotAcc,Gyro, Mag} = process_nav(Data, BodyR0),
    
    Qc = observationModel(BodyXor,Acc,Mag,BodyR0),
    io:format("Qc ~p~n",[Qc]),
    io:format("Gyro ~p~n",[Gyro]),
    {_, Xor1,Por1,Xorp,UpdatedQc}=kalmanQuaternion(Acc, Mag, Gyro, Dt,BodyXor,BodyPor,BodyR0,Qc), %Acc,Mag,Gyro,Dt,BodyXor,P0,R0
    io:format("UpdatedQc ~p~n",[UpdatedQc]),
    io:format("Xor1 ~p~n",[Xor1]),
    io:format("Xorp ~p~n",[Xorp]),
    if BodyPart==arm ->
        {BodyXpos0,BodyPpos0} = {BodyXpos,BodyPpos};
    true ->
        {BodyXpos0, BodyPpos0} = kalmanPositionPredict(Acc,BodyXor, Dt,BodyXpos,BodyPpos)
    end,

    if 
    ?Debug_info ->Valpos = lists:append([Xor1,Xorp,UpdatedQc,BodyXpos0]);% DEBUG INFO
    true -> Valpos = lists:append([Xor1,BodyXpos0])
    end,

    {ok,Valpos, {T1,BodyXpos0,BodyPpos0,Xor1,Por1,BodyR0}}. %TODO chang Xorp to Xor1


%Hierachical Model with the Head as the parent but enhance Head position using body position
bodyHeadConstraint(BodyState,HeadState,HeadValpos,BodyValpos)->
    {T1Body,BodyXpos,BodyPpos,BodyOr,BodyPor,BodyR0}=BodyState,
    {T1Head,HeadXpos,HeadPpos,HeadOr,HeadPor,BodyR0}=HeadState,
    % Body Xpos in the local frame bodyXpos is "BodyHeadDistance" in Y direction from the head
    %only use the body Quaternion ?????
    BodyHeadDistanceVector=[[0],[?BodyHeadDistance],[0],[0]], % In quaternion form
    HeadBodyDistanceVector= [[0],[-?BodyHeadDistance],[0],[0]], % In quaternion form
    BodyHeadDistanceVectorG= quatTransfomation(BodyOr,BodyHeadDistanceVector),% To global frame ??
    HeadBodyDistanceVectorG= quatTransfomation(BodyOr,HeadBodyDistanceVector),% BOth on Body frame

    HeadBodyDistanceVectorG3D= fromQuaternionTo3D(HeadBodyDistanceVectorG),
    BodyHeadDistanceVectorG3D= fromQuaternionTo3D(BodyHeadDistanceVectorG),
    [[XBody],[_],[_],[YBody],[_],[_],[ZBody],[_],[_]]=BodyXpos,
    HeadMeasurementPos= mat:'+'([[XBody],[YBody],[ZBody]],BodyHeadDistanceVectorG3D),
    R = mat:diag([?VAR_R,?VAR_R,?VAR_R]),
    io:format("HeadMeasurementPos ~p~n",[HeadMeasurementPos]),
    io:format("R ~p~n",[R]),
    io:format("HeadXpos ~p~n",[HeadXpos]),
    io:format("HeadPpos ~p~n",[HeadPpos]),
    H= [[1,0,0,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,1,0,0]],
    {HeadXposUpdated,HeadPposUpdated}=kalman:kf_update({HeadXpos,HeadPpos},H,R,HeadMeasurementPos), %H,R,HeadMeasurementPos
    [[HeadXUpdated],[_],[_],[HeadYUpdated],[_],[_],[HeadZUpdated],[_],[_]]=HeadXposUpdated,
    BodyMeasurementPos= mat:'+'([[HeadXUpdated],[HeadYUpdated],[HeadZUpdated]],HeadBodyDistanceVectorG3D),
    io:format("HeadposUpdated ~p~n",[HeadXposUpdated]),
    %Note this line is useless if mat:diag([0,0,0,0]) is used !
    % {BodyXposUpdated,BodyPposUpdated}=kalman:kf_update({BodyXpos,BodyPpos},H,mat:diag([0,0,0]),BodyMeasurementPos), %H,R,BodyOr STrict constraint
    {BodyXposUpdated,BodyPposUpdated}=noNoiseUpdate(BodyXpos,BodyPpos,BodyMeasurementPos), %H,R,BodyOr Strict constraint
    if 
        ?Debug_info ->
        io:format("HeadValpos ~p~n",[lists:sublist(HeadValpos,12)]),
        HeadValpos1 = lists:append([lists:sublist(HeadValpos,12),HeadXposUpdated]),

        BodyValpos1 = lists:append([lists:sublist(BodyValpos,12),BodyXposUpdated]);% DEBUG INFO
        true -> 
        BodyValpos1 = lists:append([BodyOr,BodyXpos]),
        HeadValpos1 = lists:append([HeadOr,HeadXpos])
    end,
    {{T1Head,HeadXposUpdated,HeadPposUpdated,HeadOr,HeadPor,BodyR0},{T1Body,BodyXposUpdated,BodyPposUpdated,BodyOr,BodyPor,BodyR0},HeadValpos1,BodyValpos1}.
headArmConstraint(HeadState,ArmState,Arm)->
    {T1Arm,ArmXpos,ArmPpos,ArmOr,ArmPor,ArmR0}=ArmState,
    {_,HeadXpos,HeadPpos,HeadOr,HeadPor,HeadR0}=HeadState,
    [[HeadX],[_],[_],[HeadY],[_],[_],[HeadZ],[_],[_]]=HeadXpos,
    if Arm == left ->
        HeadCenterDistanceVector=[[0],[0],[?HeadArmDistance],[0]]; % In quaternion form
       Arm == right ->
        HeadCenterDistanceVector=[[0],[0],[-?HeadArmDistance],[0]]; % In quaternion form
       true -> io:format("Error in arm name"),
         HeadCenterDistanceVector=[0.15,0,0,0]
    end,
    HeadCenterDistanceVectorG= fromQuaternionTo3D(quatTransfomation(HeadOr,HeadCenterDistanceVector)),% To global frame 
    ArmCenterPos= mat:'+'([[HeadX],[HeadY],[HeadZ]],HeadCenterDistanceVectorG),
    
    %Get the direction of the arm
    Y=[[0],[-?ArmCenterBiceps],[0],[0]],
    YG= fromQuaternionTo3D(quatTransfomation(ArmOr,Y)),
    %Get the arm position
    ArmMeasurementPos= mat:'+'(ArmCenterPos, YG),


    H= [[1,0,0,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,1,0,0]],
    % {ArmMeasurementPosUpdated,ArmeasurementPpos}=kalman:kf_update({ArmXpos,ArmPpos},H,mat:diag([0,0,0]),ArmMeasurementPos),
    {ArmMeasurementPosUpdated,ArmeasurementPpos}=noNoiseUpdate(ArmXpos,ArmPpos,ArmMeasurementPos),
    % {ArmMeasurementPosUpdated,ArmCenterPpos}.
    if ?Debug_info ->ArmValpos = lists:append([ArmOr,ArmOr,ArmOr,ArmMeasurementPosUpdated]);% DEBUG INFO
    true -> ArmValpos = lists:append([ArmOr,ArmMeasurementPosUpdated])
    end,
    io:format("ArmMeasurementPosUpdated ~p~n",[ArmMeasurementPosUpdated]),
    {{T1Arm,ArmMeasurementPosUpdated,ArmeasurementPpos,ArmOr,ArmPor,ArmR0},ArmValpos}.
armForearmConstraint({_,ArmXpos,_,ArmOr,_,_},{T1Forearm,ForearmXpos,ForearmPpos,ForearmOr,ForearmPor,ForearmR0}) ->
    [[ArmX],[_],[_],[ArmY],[_],[_],[ArmZ],[_],[_]]=ArmXpos,
    BicpesToElbow= [[0],[-?BicepsElbow],[0],[0]],
    BicpesToElbowG= quatTransfomation(ArmOr,BicpesToElbow),
  
    ElbowPos= mat:'+'([[ArmX],[ArmY],[ArmZ]],fromQuaternionTo3D(BicpesToElbowG)),

    %Get direction of the forearm
    Y=[[0],[-?ElbowWrist],[0],[0]],
    YG= fromQuaternionTo3D(quatTransfomation(ForearmOr,Y)),

    WristPos= mat:'+'(ElbowPos, YG),
    H= [[1,0,0,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,1,0,0]], 
    io:format("WristPos ~p~n",[WristPos]),
    io:format("ForearmXpos ~p~n",[ForearmXpos]),
    {WristPosUpdated,ElbowPpos}=noNoiseUpdate(ForearmXpos,ForearmPpos,WristPos),
    if ?Debug_info ->ForearmValpos = lists:append([ForearmOr,ForearmOr,ForearmOr,WristPosUpdated]);% DEBUG INFO
    true -> ForearmValpos = lists:append([ForearmOr,WristPosUpdated])
    end,
    {{T1Forearm,WristPosUpdated,ElbowPpos,ForearmOr,ForearmPor,ForearmR0},ForearmValpos}.
conjugateQuaternion([[Q0],[Q1],[Q2],[Q3]]) ->
    [[Q0],[-Q1],[-Q2],[-Q3]].
fromQuaternionTo3D([[_],[X],[Y],[Z]]) ->
    [[X],[Y],[Z]].
noNoiseUpdate(Xpos,Ppos,MeasurementPos)->
    [[X],[Y],[Z]]=MeasurementPos,
    [[_],[Vx],[Ax],[_],[Vy],[Ay],[_],[Vz],[Az]]=Xpos,
    {[[X],[Vx],[Ax],[Y],[Vy],[Ay],[Z],[Vz],[Az]],Ppos}. % return the updated position
