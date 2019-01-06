%the function is to arrange transition course
function [sys,x0,str,ts]=fun_Vrep_models(t,x,u,flag)
switch flag,
    case 0,
        [sys,x0,str,ts]=mdlInit();
    case 1,
        sys=mdlDerivatives(t,x,u);
    case 3,
        sys=mdlOutput(t,x,u);
    case 9
        sys=mdlTerminate(t,x,u);
    case { 2, 4}
        sys = []; % 未使用的flag值
    otherwise
        error(['Unhandled flag = ',num2str(flag)]); % 处理错误
end;

function [sys,x0,str,ts]= mdlInit(t,x,u)
global clientID vrep 
size=simsizes;
size.NumContStates=0;
size.NumDiscStates=0;
size.NumOutputs=16;%实际倾转角4、姿态角3、角速度3、位置3、速度3,实际力矩3
size.NumInputs=11;%倾转电机4+推力电机4+位置期望3
size.DirFeedthrough=1;
size.NumSampleTimes=1;
sys=simsizes(size);
x0=[];
str=[];
ts=[0,1];%不进行微步计算
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',-19997,true,true,5000,5);
if (clientID>-1)
        disp('Connected to remote API server');
        % enable the synchronous mode on the client:
        vrep.simxSynchronous(clientID,true);
        % start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%启动vrep仿真
        
        % 流模式
        vrep.simxReadStringStream(clientID,'sensorMy',vrep.simx_opmode_streaming); 
else
        disp('Failed connecting to remote API server');
end

function sys=mdlDerivatives(t,x,u)
sys = [];

function sys=mdlOutput(t,x,u)
global clientID vrep
if (clientID>-1)
%% main code
    f1=1.0260e-05*u(1)^2;
    f2=1.0260e-05*u(2)^2;
    f3=1.0260e-05*u(3)^2;
    f4=1.0260e-05*u(4)^2;
    Force_d=-[f1 f3 f4 f2];
%     Force_d=[u(1) u(2) u(3) u(4)];
    Tilt_d=[u(5) u(7) u(8) u(6)];
    pos_d=[u(9) u(10) u(11)];
%     Force_d=-1.02*9.7643*1/4*ones(1,4);
%     Tilt_d=[0 0 0 0];
%     if t>1
%         Tilt_d=[10*pi/180,-10*pi/180,0,0];
%     end
%     pos_d=[0 0 0];
    %pack data
    command=[Force_d,Tilt_d,pos_d];
    [commandData]=vrep.simxPackFloats(command);
   
    %sendingdata
    [returnCode]=vrep.simxWriteStringStream(clientID,'myCommands',commandData,vrep.simx_opmode_oneshot);    
  
    %trigger
    [returnCode]=vrep.simxSynchronousTrigger(clientID);%simulink最初两个时间步没有计时，导致vrep时间比simulink提前0.01s,即控制器时延0.01s
    vrep.simxGetPingTime(clientID);%call a random function in a blocking manner to make sure simulaiton step finished computing on vrep
    vrep.simxGetPingTime(clientID);
    
    %reading data
    [err,ssd]=vrep.simxReadStringStream(clientID,'sensorMy',vrep.simx_opmode_buffer);  
     if (err == 0)
        sensor=vrep.simxUnpackFloats(ssd);
        sensor=double(sensor);%single转换为double,删除会报错
%        display('t')
    else
        sensor=zeros(1,16);
        display('f')
    end       
    
    pos_e=sensor(1:3);
    vel_e=sensor(4:6);
    eular=sensor(7:9);
    w_e=sensor(10:12);
    tilt_real=[sensor(13) sensor(16) sensor(14) sensor(15)];   
    phi=eular(1);theta=eular(2);psi=eular(3);
    Rbe_arate=[1  , 0          , -sin(theta)
            0  , cos(phi)   , cos(theta)*sin(phi)
            0  , -sin(phi)  , cos(theta)*cos(phi)];   
    w_b=w_e*Rbe_arate;
end
sys=[pos_e , vel_e , eular , w_b , tilt_real];

function sys=mdlTerminate(t,x,u)
global clientID vrep 
    %getHandle
    [returnCode,frontTiltHandle]=vrep.simxGetObjectHandle(clientID,'joint_tilt_front', vrep.simx_opmode_blocking);%倾转电机句柄
    [returnCode,backTiltHandle]=vrep.simxGetObjectHandle(clientID,'joint_tilt_back', vrep.simx_opmode_blocking);
    [returnCode,leftTiltHandle]=vrep.simxGetObjectHandle(clientID,'joint_tilt_left', vrep.simx_opmode_blocking);
    [returnCode,rightTiltHandle]=vrep.simxGetObjectHandle(clientID,'joint_tilt_right', vrep.simx_opmode_blocking);
    
    %reset tilt angle
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,frontTiltHandle,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,backTiltHandle,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,leftTiltHandle,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,rightTiltHandle,0,vrep.simx_opmode_blocking);

    %trigger    
    [returnCode]=vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);
   
    % stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking); 
    
    % close the connection to V-REP:
    vrep.simxFinish(-1);
    
    % call the destructor
    vrep.delete(); 
    disp('Program ended')

sys = [];

        

