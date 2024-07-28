-module(e3).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).
-export([observationModel/3]).

-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).
-define(Debug_info, false).
-define(BodyHeadDistance, 0.30).
-define(HeadArmDistance, 0.25).
-define(ArmCenterBiceps, 0.15).
-define(BicepsElbow, 0.15).
-define(ElbowWrist, 0.15).

-record(cal, {gyro, mag}).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("Place de pmod_nav flat and still!~n"),
    [Gx,Gy,Gz] = calibrate(acc, [out_x_g,out_y_g,out_z_g], 300),
    [Mx1,My1,Mz1] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    _ = io:get_line("Turn the pmod_nav 180° around the z axis then press enter"),
    [Mx2,My2,_] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    _ = io:get_line("Turn the pmod_nav 180° around the x axis then press enter"),
    [_,_,Mz2] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    MBx = 0.5*(Mx1+Mx2),
    MBy = 0.5*(My1+My2),
    MBz = 0.5*(Mz1+Mz2),
       
    _ = io:get_line("Place the pmod_nav at 0° then press enter"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 100),
    [Mx,My,Mz] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    R0 = ahrs([Ax,Ay,Az], [(Mx-MBx),My-MBy,(Mz-MBz)]), % why minus? ?? TODO
    io:format("Calibration matrice at calibration e2: ~p~n", [mat:tr(R0)]),
    {mat:tr(R0),#cal{gyro={Gx,Gy,Gz}, mag={MBx,MBy,MBz}}}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init({R0,C=#cal{gyro={GBx,GBy,GBz}, mag={MBx,MBy,MBz}}}) ->
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
    HeadOr= dcm2quat(numerl_matrix_to_erl_matrix(R0)),
    io:format("HeadOr ~p~n",[HeadOr]),
    HeadPor = mat:diag([10,10,10,10]),
    HeadXpos = {T0,HeadPos, HeadPpos, HeadOr, HeadPor, R0},

    
    BodyPosition = quatTransfomation(HeadOr,[[-?BodyHeadDistance],[0],[0]]),
    [[Bodyx],[Bodyy],[Bodyz]]=BodyPosition,
    BodyPos = [
        [Bodyx],[0],[0], % Position Velocity acceleration
        [Bodyy],[0],[0], 
        [Bodyz],[0],[0]], 
    BodyPpos = mat:eye(9),
    BodyOr= dcm2quat(numerl_matrix_to_erl_matrix(R0)),

    BodyPor = mat:diag([10,10,10,10]),
    BodyXpos = {T0,BodyPos, BodyPpos, BodyOr, BodyPor, R0},

    ArmPosition = quatTransfomation(HeadOr,[[-?HeadArmDistance],[-?ArmCenterBiceps],[0]]),
    [[Armx],[Army],[Armz]]=ArmPosition,
    ArmPos=[[Armx],[0],[0], % Position LeftArm
        [Army],[0],[0], % V LeftArm
        [Armz],[0],[0]], % Acc LeftArm
    ArmPpos = mat:eye(9),
    ArmOr= dcm2quat(numerl_matrix_to_erl_matrix(R0)),
    ArmPor = mat:diag([10,10,10,10]),
    ArmXpos = {T0,ArmPos, ArmPpos, ArmOr, ArmPor, R0},
    
    ForearmPosition= quatTransfomation(HeadOr,[[-?HeadArmDistance],[-?ArmCenterBiceps-?BicepsElbow],[0]]),
    [[Forearmx],[Forearmy],[Forearmz]]=ForearmPosition,
    ForearmPos=[[Forearmx],[0],[0], % Position Forearm
        [Forearmy],[0],[0], % V Forearm
        [Forearmz],[0],[0]], % Acc Forearm
    ForearmPpos = mat:eye(9),
    ForearmOr= dcm2quat(numerl_matrix_to_erl_matrix(R0)),
    ForearmPor = mat:diag([10,10,10,10]),
    ForearmXpos = {T0,ForearmPos, ForearmPpos, ForearmOr, ForearmPor, R0},

    State = {HeadXpos,BodyXpos,ArmXpos,ForearmXpos,C},

    {ok, State, Spec}.


measure({HeadXpos,BodyXpos,ArmXpos,ForearmXpos,C}) -> 

    LoopCount = 100,
    Host = lists:nthtail(10, atom_to_list(node())),
    IsBody = lists:prefix("body", Host),
    IsHead = lists:prefix("head", Host),
    IsArm = lists:prefix("arm", Host),
    IsForearm = lists:prefix("forearm", Host),
    % {T1,BodyState,HeadState,ArmState,ForearmState}=fromSensorDataToState(readNewestData(IsBody,IsHead,IsArm,IsForearm)),
    T1=hera:timestamp(),
    if IsHead -> 
        {_,HeadValpos,UpdatedHeadXpos}=loop(LoopCount, HeadXpos,C),
        UpdatedBodyXpos=BodyXpos,
        UpdatedArmXpos=ArmXpos,
        UpdatedForearmXpos=ForearmXpos,
        {ConstraintHeadXpos,ConstraintBodyXpos,ConstraintHeadValPos,ConstraintBodyValPos}=bodyHeadConstraint(UpdatedBodyXpos,UpdatedHeadXpos,HeadValpos,stateToValpos(UpdatedBodyXpos)),
        {ConstraintArmXpos,ConstraintArmValpos}=headArmConstraint(ConstraintHeadXpos,UpdatedArmXpos,right),
        {ConstraintForearmXpos,ConstraintForearmValpos}=armForearmConstraint(ConstraintArmXpos,UpdatedForearmXpos),       
        Valpos = lists:append([[T1],ConstraintHeadValPos,ConstraintBodyValPos,ConstraintArmValpos,ConstraintForearmValpos]),
        {ok,Valpos, {ConstraintHeadXpos,ConstraintBodyXpos,ConstraintArmXpos,ConstraintForearmXpos,C}};        
    IsBody -> 
        {_,BodyValpos,UpdatedBodyXpos}=loop(LoopCount, BodyXpos,C),
        UpdatedHeadXpos=HeadXpos,
        UpdatedArmXpos=ArmXpos,
        UpdatedForearmXpos=ForearmXpos,
        {ConstraintHeadXpos,ConstraintBodyXpos,ConstraintHeadValPos,ConstraintBodyValPos}=bodyHeadConstraint(UpdatedBodyXpos,UpdatedHeadXpos,stateToValpos(UpdatedHeadXpos),BodyValpos),
        {ConstraintArmXpos,ConstraintArmValpos}=headArmConstraint(ConstraintHeadXpos,UpdatedArmXpos,right),
        {ConstraintForearmXpos,ConstraintForearmValpos}=armForearmConstraint(ConstraintArmXpos,UpdatedForearmXpos),
        Valpos = lists:append([[T1],ConstraintHeadValPos,ConstraintBodyValPos,ConstraintArmValpos,ConstraintForearmValpos]),
        io:format("TimeHead ~p~n",[hera:timestamp()-T1]),
        {ok,Valpos, {ConstraintHeadXpos,ConstraintBodyXpos,ConstraintArmXpos,ConstraintForearmXpos,C}};
    IsArm -> 
        {_,ArmValpos,UpdatedArmXpos}=loop(LoopCount, ArmXpos,C),
        UpdatedHeadXpos=HeadXpos,
        UpdatedBodyXpos=BodyXpos,
        UpdatedForearmXpos=ForearmXpos,
        {ConstraintArmXpos,ConstraintArmValpos}=headArmConstraint(UpdatedHeadXpos,UpdatedArmXpos,right),
        {ConstraintForearmXpos,ConstraintForearmValpos}=armForearmConstraint(ConstraintArmXpos,UpdatedForearmXpos),
        Valpos = lists:append([[T1],stateToValpos(UpdatedHeadXpos),stateToValpos(UpdatedBodyXpos),ConstraintArmValpos,ConstraintForearmValpos]),
        io:format("TimeArm ~p~n",[hera:timestamp()-T1]),
        {ok,Valpos, {UpdatedHeadXpos, UpdatedBodyXpos, ConstraintArmXpos,ConstraintForearmXpos,C}};        
    IsForearm -> 
        io:format("IsForearm ~p~n",[IsForearm]),
        {_,_,UpdatedForearmXpos}=loop(LoopCount, ForearmXpos,C),
        UpdatedHeadXpos=HeadXpos,
        UpdatedBodyXpos=BodyXpos,
        UpdatedArmXpos=ArmXpos,
        {ConstraintForearmXpos,ConstraintForearmValpos}=armForearmConstraint(UpdatedArmXpos,UpdatedForearmXpos),
        Valpos = lists:append([[T1],stateToValpos(UpdatedHeadXpos),stateToValpos(UpdatedBodyXpos),stateToValpos(UpdatedArmXpos),ConstraintForearmValpos]),
        io:format("TimeForearm ~p~n",[hera:timestamp()-T1]),
        {ok,Valpos, {UpdatedHeadXpos, UpdatedBodyXpos, UpdatedArmXpos,ConstraintForearmXpos,C}};
    true -> {undefined, {HeadXpos,BodyXpos,ArmXpos,ForearmXpos,C}}
    end.

loop(0, HeadXpos,_C) ->
    {_T0Head,HeadPos,_HeadPpos,HeadOr,_HeadPor,_HeadR0}=HeadXpos,
    T1=hera:timestamp(), 
    if 
    ?Debug_info ->Valpos = lists:append([[T1],HeadOr,HeadOr,HeadOr,HeadPos]);% DEBUG INFO
    true -> Valpos = lists:append([HeadOr,HeadPos])
    end,
    {ok,Valpos, HeadXpos}; 
loop(N, HeadXpos,C=#cal{gyro={GBx,GBy,GBz},mag={MBx,MBy,MBz}}) ->
    {T0Head,HeadPos,HeadPpos,HeadOr,HeadPor,HeadR0}=HeadXpos, 
    [Ax, Ay, Az, Gx, Gy, Gz] = pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl, out_x_g, out_y_g, out_z_g]),
    Acc = [Ax, Ay, -Az],
    Gyro = [Gx - GBx, Gy - GBy, -(Gz - GBz)],
    [Mx, My, Mz] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    Data = lists:append([
        scale(Acc, 9.81),
        scale(Gyro, math:pi() / 180),
        [-(Mx - MBx), My - MBy, -(Mz - MBz)]
    ]),
    T1Head=hera:timestamp(),
    Dt=(T1Head-T0Head)/1000,
    {HeadAcc,_,HeadGyro,HeadMag}=process_nav([Data], HeadR0),
    Qc = observationModel(HeadOr,HeadAcc,HeadMag),
    {_, Xor1,Por1,_Xorp,_UpdatedQc}=kalmanQuaternion(HeadAcc, HeadMag, HeadGyro, Dt,HeadOr,HeadPor,HeadR0,Qc), %Acc,Mag,Gyro,Dt,BodyXor,P0,R0
    {HeadPos0, HeadPpos0} = kalmanPositionPredict(HeadAcc,HeadOr, Dt,HeadPos,HeadPpos),

    HeadXpos1={T1Head,HeadPos0,HeadPpos0,Xor1,Por1,HeadR0}, 
    loop(N - 1, HeadXpos1,C).
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
    {Acc, [], Gyro, Mag}. 

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
    Omega =numerl:matrix([ 
        [0,Wx,Wy,Wz],
        [-Wx,0,-Wz,Wy],
        [-Wy,Wz,0,-Wx],
        [-Wz,-Wy,Wx,0]
    ]),
    F = mat:'+'(mat:eye(4), mat:'*'(0.5*Dt, Omega)),
    Q = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
    H = mat:eye(4),
    R = mat:diag([?VAR_R,?VAR_R,?VAR_R,?VAR_R]),
    Quaternion= numerl:matrix(conjugateQuaternion(BodyXor)),
    {Xorp, Porp} = kalman:kf_predict({Quaternion,P0}, F, Q),
    UnitXorp=unit(transpose(numerl:mtfl(Xorp))),
    {Xor1, Por1} = case qdot(Quat, UnitXorp) > 0 of
        true ->
            Qc= Quat,
            kalman:kf_update({Xorp, Porp}, H, R, numerl:matrix(Quat));
        false ->
            Qc= transpose(numerl:mtfl(mat:'*'(-1,numerl:matrix(Quat)))),
            kalman:kf_update({Xorp, Porp}, H, R, numerl:matrix(Qc))            
        end,
        


    UnitXor1 = unit(conjugateQuaternion(transpose(numerl:mtfl(Xor1)))),
    {ok, UnitXor1,Por1,UnitXorp,Qc}.

kalmanPositionPredict(Acc,Q,Dtpos,Pos,Ppos) ->
    Fpos = numerl:matrix([
        [1,Dtpos,(Dtpos*Dtpos)/2,0,0,0,0,0,0], % North
        [0,1,Dtpos,0,0,0,0,0,0], % V_North
        [0,0,1,0,0,0,0,0,0], % Acc_North
        [0,0,0,1,Dtpos,(Dtpos*Dtpos)/2,0,0,0], % UP
        [0,0,0,0,1,Dtpos,0,0,0], % V_UP
        [0,0,0,0,0,1,0,0,0], % Acc_UP
        [0,0,0,0,0,0,1,Dtpos,(Dtpos*Dtpos)/2], % EAST
        [0,0,0,0,0,0,0,1,Dtpos], % V_EAST
        [0,0,0,0,0,0,0,0,1] % Acc_EAST
    ]),
    Qpos = mat:diag([?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL]),
    [[X],[Vx],[_],[Y],[Vy],[_],[Z],[Vz],[_]] = Pos,
    AccTrans = quatTransfomation(Q,transpose(Acc)),
    [[AccNorth],[AccUp],[AccEast]] = AccTrans,
    NewPos= numerl:matrix([[X],[Vx],[AccNorth],[Y],[Vy],[AccUp-9.81],[Z],[Vz],[AccEast]]),

    {Xpos0, Ppos0} = kalman:kf_predict({NewPos,Ppos}, Fpos, Qpos),
    {transpose(numerl:mtfl(Xpos0)), Ppos0}.

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
    P_quat= [[0]|P],
    Fp= q_product(Q,q_product(P_quat,Qinv)),
    tl(Fp).

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
    VnorthL= quatTransfomation(Qa,[[1],[0],[0]]),
    Vmxyz= quatTransfomation(Qainv,[[Vmx],[Vmy],[Vmz]]),
    [[Bx],_,[Bz]]= Vmxyz,
    Vmxz= [[Bx/math:sqrt(Bx*Bx+Bz*Bz)],[0],[Bz/math:sqrt(Bx*Bx+Bz*Bz)]],

    VmxzL= quatTransfomation(Qa,Vmxz),
    [[VmxzLx],[VmxzLy],[VmxzLz]]=VmxzL,
    [[VnorthLx],[VnorthLy],[VnorthLz]]=VnorthL,
    Domegam= math:acos(dot([VnorthLx,VnorthLy,VnorthLz],[VmxzLx,VmxzLy,VmxzLz])),
    [Qme2,Qme3,Qme4]=consttimesVector(math:sin(Domegam/2),cross_product([VnorthLx,VnorthLy,VnorthLz],[VmxzLx,VmxzLy,VmxzLz])),
    Qme1= math:cos(Domegam/2),
    Qme = [[Qme1],[Qme2],[Qme3],[Qme4]],
    Qm=q_product(Qme,Qa),
    unit(Qm).



%Hierachical Model with the Head as the parent but enhance Head position using body position
bodyHeadConstraint(BodyState,HeadState,HeadValpos,BodyValpos)->
    {T1Body,BodyXpos,BodyPpos,BodyOr,BodyPor,BodyR0}=BodyState,
    {T1Head,HeadXpos,HeadPpos,HeadOr,HeadPor,BodyR0}=HeadState,
    % Body Xpos in the local frame bodyXpos is "BodyHeadDistance" in Y direction from the head
    %only use the body Quaternion ?????
    BodyHeadDistanceVector=[[?BodyHeadDistance],[0],[0]], % In quaternion form
    HeadBodyDistanceVector= [[-?BodyHeadDistance],[0],[0]], % In quaternion form
    BodyHeadDistanceVectorG= quatTransfomation(BodyOr,BodyHeadDistanceVector),% To global frame ??
    HeadBodyDistanceVectorG= quatTransfomation(BodyOr,HeadBodyDistanceVector),% BOth on Body frame

    [[XBody],[_],[_],[YBody],[_],[_],[ZBody],[_],[_]]=BodyXpos,
    HeadMeasurementPos= mat:'+'(numerl:matrix([[XBody],[YBody],[ZBody]]),numerl:matrix(BodyHeadDistanceVectorG)),
    R = mat:diag([?VAR_R,?VAR_R,?VAR_R]),
    H= numerl:matrix([[1,0,0,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,1,0,0]]),
    {HeadXposUpdated,HeadPposUpdated}=kalman:kf_update({numerl:matrix(HeadXpos),HeadPpos},H,R,HeadMeasurementPos), %H,R,HeadMeasurementPos
    [[HeadXUpdated],[_],[_],[HeadYUpdated],[_],[_],[HeadZUpdated],[_],[_]]=transpose(numerl:mtfl(HeadXposUpdated)),
    BodyMeasurementPos= transpose(numerl:mtfl(mat:'+'(numerl:matrix([[HeadXUpdated],[HeadYUpdated],[HeadZUpdated]]),numerl:matrix(HeadBodyDistanceVectorG)))),
    %Note this line is useless if mat:diag([0,0,0,0]) is used !
    % {BodyXposUpdated,BodyPposUpdated}=kalman:kf_update({BodyXpos,BodyPpos},H,mat:diag([0,0,0]),BodyMeasurementPos), %H,R,BodyOr STrict constraint
    {BodyXposUpdated,BodyPposUpdated}=noNoiseUpdate(BodyXpos,BodyPpos,BodyMeasurementPos), %H,R,BodyOr Strict constraint
    if 
        ?Debug_info ->
        HeadValpos1 = lists:append([lists:sublist(HeadValpos,12),transpose(numerl:mtfl(HeadXposUpdated))]),
        BodyValpos1 = lists:append([lists:sublist(BodyValpos,12),BodyXposUpdated]);% DEBUG INFO
        true -> 
        BodyValpos1 = lists:append([BodyOr,BodyXpos]),
        HeadValpos1 = lists:append([HeadOr,HeadXpos])
    end,
    {{T1Head,transpose(numerl:mtfl(HeadXposUpdated)),HeadPposUpdated,HeadOr,HeadPor,BodyR0},{T1Body,BodyXposUpdated,BodyPposUpdated,BodyOr,BodyPor,BodyR0},HeadValpos1,BodyValpos1}.
headArmConstraint(HeadState,ArmState,Arm)->
    {T1Arm,ArmXpos,ArmPpos,ArmOr,ArmPor,ArmR0}=ArmState,
    {_,HeadXpos,_HeadPpos,HeadOr,_HeadPor,_HeadR0}=HeadState,
    [[HeadX],[_],[_],[HeadY],[_],[_],[HeadZ],[_],[_]]=HeadXpos,
    if Arm == left ->
        HeadCenterDistanceVector=[[0],[?HeadArmDistance],[0]]; % In quaternion form
       Arm == right ->
        HeadCenterDistanceVector=[[0],[-?HeadArmDistance],[0]]; % In quaternion form
       true -> io:format("Error in arm name"),
         HeadCenterDistanceVector=[0.15,0,0,0]
    end,
    HeadCenterDistanceVectorG= quatTransfomation(HeadOr,HeadCenterDistanceVector),% To global frame 
    ArmCenterPos= mat:'+'(numerl:matrix([[HeadX],[HeadY],[HeadZ]]),numerl:matrix(HeadCenterDistanceVectorG)),
    
    %Get the direction of the arm
    Y=[[-?ArmCenterBiceps],[0],[0]],
    YG= quatTransfomation(ArmOr,Y),
    %Get the arm position
    ArmMeasurementPos= mat:'+'(ArmCenterPos, numerl:matrix(YG)),

    H= numerl:matrix([[1,0,0,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,1,0,0]]),
    % {ArmMeasurementPosUpdated,ArmeasurementPpos}=kalman:kf_update({ArmXpos,ArmPpos},H,mat:diag([0,0,0]),ArmMeasurementPos),
    {ArmMeasurementPosUpdated,ArmeasurementPpos}=noNoiseUpdate(ArmXpos,ArmPpos,transpose(numerl:mtfl(ArmMeasurementPos))),
    % {ArmMeasurementPosUpdated,ArmCenterPpos}.
    if ?Debug_info ->ArmValpos = lists:append([ArmOr,ArmOr,ArmOr,ArmMeasurementPosUpdated]);% DEBUG INFO
    true -> ArmValpos = lists:append([ArmOr,ArmMeasurementPosUpdated])
    end,
    {{T1Arm,ArmMeasurementPosUpdated,ArmeasurementPpos,ArmOr,ArmPor,ArmR0},ArmValpos}.
armForearmConstraint({_,ArmXpos,_,ArmOr,_,_},{T1Forearm,ForearmXpos,ForearmPpos,ForearmOr,ForearmPor,ForearmR0}) ->
    [[ArmX],[_],[_],[ArmY],[_],[_],[ArmZ],[_],[_]]=ArmXpos,
    BicpesToElbow= [[-?BicepsElbow],[0],[0]],
    BicpesToElbowG= quatTransfomation(ArmOr,BicpesToElbow),
  
    ElbowPos= mat:'+'(numerl:matrix([[ArmX],[ArmY],[ArmZ]]),numerl:matrix(BicpesToElbowG)),

    %Get direction of the forearm
    Y=[[-?ElbowWrist],[0],[0]],
    YG= quatTransfomation(ForearmOr,Y),

    WristPos= mat:'+'(ElbowPos, numerl:matrix(YG)),
    H= [[1,0,0,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,1,0,0]], 
    {WristPosUpdated,ElbowPpos}=noNoiseUpdate(ForearmXpos,ForearmPpos,transpose(numerl:mtfl(WristPos))),
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


% Read the data from all the sensor, select the newest data
getNewestSensorData(Data1,Data2,Data3)->
    {T1Data1,_}=Data1,
    {T1Data2,_}=Data2,
    {T1Data3,_}=Data3,
    if T1Data1 > T1Data2 and (T1Data1 > T1Data3) ->
        Data1;
    (T1Data2 > T1Data1 and (T1Data2 > T1Data3)) ->
        Data2;
    (T1Data3 > T1Data1 and (T1Data3 > T1Data2)) ->
        Data3
    end.
fromSensorDataToState(Data)->
    {T1,BodyHeadArmForerarmData}=lists:split(1,Data),
    {HeadState,BodyHaedArmForArmData}=lists:split(7,BodyHeadArmForerarmData),
    {BodyState,ArmForearmData}=lists:split(7,BodyHaedArmForArmData),
    {ArmState,ForearmState}=lists:split(7,ArmForearmData),
    {T1,BodyState,HeadState,ArmState,ForearmState}.
readNewestData(IsBody,IsHead,IsArm,IsForearm)->
    if IsBody==true->
        DataHead = hera_data:get(e3, body_head@head),
        HeadSensorData=fromDataTostate(DataHead),
        DataArm = hera_data:get(e3, body_head@arm),
        ArmSensorData=fromDataTostate(DataArm),
        Dataforearm = hera_data:get(e3, body_head@forearm),
        ForearmSensorData=fromDataTostate(Dataforearm),
        getNewestSensorData(HeadSensorData,ArmSensorData,ForearmSensorData);
    IsHead==true->
        DataBody = hera_data:get(e3, body_head@body),
        BodySensorData=fromDataTostate(DataBody),
        DataArm = hera_data:get(e3, body_head@arm),
        ArmSensorData=fromDataTostate(DataArm),
        Dataforearm = hera_data:get(e3, body_head@forearm),
        ForearmSensorData=fromDataTostate(Dataforearm),
        getNewestSensorData(BodySensorData,ArmSensorData,ForearmSensorData);
    IsArm==true->
        DataBody = hera_data:get(e3, body_head@body),
        BodySensorData=fromDataTostate(DataBody),
        DataHead = hera_data:get(e3, body_head@head),
        HeadSensorData=fromDataTostate(DataHead),
        Dataforearm = hera_data:get(e3, body_head@forearm),
        ForearmSensorData=fromDataTostate(Dataforearm),
        getNewestSensorData(BodySensorData,HeadSensorData,ForearmSensorData);
    IsForearm==true->
        DataBody = hera_data:get(e3, body_head@body),
        BodySensorData=fromDataTostate(DataBody),
        DataHead = hera_data:get(e3, body_head@head),
        HeadSensorData=fromDataTostate(DataHead),
        DataArm = hera_data:get(e3, body_head@arm),
        ArmSensorData=fromDataTostate(DataArm),
        getNewestSensorData(BodySensorData,HeadSensorData,ArmSensorData);
    true -> io:format("Error in node name")
    end.


% Data is in the form HeadOr,HeadPos,BodyOr,BodyPos,ArmOr,ArmPos,ForearmOr,ForearmPos
fromDataTostate(Data)->
    {Times,HeadBodyArmForearmData}=lists:split(3,Data),
    {_,_,T1}=Times,
    % select the first 7 elements of the list
    [HeadData,BodyArmForearmData]=lists:split(7,HeadBodyArmForearmData),
    % select the 8th to 15th elements of the list
    [BodyData,ArmForearmData]=lists:split(7,BodyArmForearmData),
    % select the 16th to 23th elements of the list
    [ArmData,ForearmData]=lists:split(7,ArmForearmData),
    {T1,HeadData,BodyData,ArmData,ForearmData}.

stateToValpos(State)->
    % {T1,pos,Ppos0,Or,Por,R0}
    {_,Pos,Ppos,Or,_,_}=State,
    if ?Debug_info ->Valpos = lists:append([Or,Or,Or,Pos]);% DEBUG INFO
    true -> Valpos = lists:append([Or,Pos])
    end,
    Valpos.
numerl_matrix_to_erl_matrix(Numerl_matrix) ->
    {_,N,_M,_}=Numerl_matrix,
    Array=mat:to_array(Numerl_matrix),
    numerl_matrix_to_erl_matrix(Array,N,[]).
numerl_matrix_to_erl_matrix([], _N, Acc)->Acc;
numerl_matrix_to_erl_matrix(List, N, Acc)->
    {Row, Rest} = lists:split(N, List),
    numerl_matrix_to_erl_matrix(Rest, N, [Row | Acc]).
transpose([]) ->
    [];
transpose([ Head|Tail]) ->
    [[Head] | transpose(Tail)].