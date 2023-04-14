%% Inicio de conexion
sim=remApi('remoteApi'); % usando el prototipo de función (remoteApiProto.m)
sim.simxFinish(-1); % Cerrar las conexiones anteriores en caso de que exista una
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
%% Verificación de la conexión
if (clientID>-1)
    disp('Conexión con Coppelia iniciada');
    %% Codigo de control
    % Preparación
    [returnCode, left_motor]=sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking);
    [returnCode, front_sensor]=sim.simxGetObjectHandle(clientID,...
        'Pioneer_p3dx_ultrasonicSensor4',sim.simx_opmode_blocking);
    % Acciones
    [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor,0.5,...
        sim.simx_opmode_blocking);
    [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,...
        front_sensor, sim.simx_opmode_streaming);
    for i = 1:200
        [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,...
        front_sensor, sim.simx_opmode_buffer);
        disp(norm(detectedPoint));
        pause(0.1)
    end
    [returnCode] = sim.simxSetJointTargetVelocity(clientID,left_motor,0,...
        sim.simx_opmode_blocking);
    disp('Conexión con Coppelia Terminada');
    sim.simxFinish(clientID);
end
sim.delete(); % Llamar al destructor!
