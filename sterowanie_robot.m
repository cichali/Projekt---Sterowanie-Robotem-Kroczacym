%% zdefiniowanie komunikacje miêdzy robotem a MatLab
chodzenie = rospublisher('/darwin/cmd_vel');
ramie_lewe = rospublisher('/darwin/j_high_arm_l_position_controller/command');
ramie_prawe = rospublisher('/darwin/j_high_arm_r_position_controller/command');
szyja = rospublisher('/darwin/j_tilt_position_controller/command');
stany = rossubscriber('/gazebo/model_states');
wiadomosc1 = rosmessage(chodzenie.MessageType);
wiadomosc2 = rosmessage(szyja.MessageType);
wiadomosc5 = rosmessage(ramie_lewe.MessageType);
wiadomosc6 = rosmessage(ramie_prawe.MessageType);
%% Ustawienie pocz¹tkowej postawy robota
wiadomosc2.Data = -0.5;
send(szyja, wiadomosc2);
pause(1);
wiadomosc2.Data = 0;
send(szyja, wiadomosc2);
wiadomosc5.Data = 1.4;
wiadomosc6.Data = 1.4;
send(ramie_lewe,wiadomosc5);
pause(0.1)
send(ramie_prawe,wiadomosc6);
pause(0.1)
wiadomosc1.Linear.X = 1;
send(chodzenie,wiadomosc1);
pause(0.5)
wiadomosc1.Linear.Y = 1;
wiadomosc1.Linear.X = 0;
send(chodzenie,wiadomosc1);
pause(0.5)
wiadomosc1.Linear.Y = 0;
send(chodzenie,wiadomosc1);
pause(0.5)
%% Sterowanie
odbierz = stany.receive; %pobranie pozycji robota
punkty = odbierz.Pose;
[m,n]=size(punkty);
robot_x = punkty(2).Position.X;
robot_y = punkty(2).Position.Y;
euler = quat2eul([punkty(2).Orientation.W punkty(2).Orientation.Z punkty(2).Orientation.Y punkty(2).Orientation.X]); %konwersja 
yaw = euler(3);
if(yaw<0)
    yaw = 2*pi+yaw;
end
x = [];
y = [];
trasa_x = robot_x; %œledzenie pozycji robota
trasa_y = robot_y;

map = binaryOccupancyMap(10,10,10); %utworzenie mapy przeszkód
rozmiar = size(punkty);

przeszkoda_x = 0;
przeszkoda_y = 0;
robotGoal = [5, 8]; % wspó³rzêdne celu robota
robotInitialLocation = [robot_x robot_y];

for przeszkoda = 3:rozmiar(1)
    
    przeszkoda_x = [przeszkoda_x; punkty(przeszkoda).Position.X]; %pobranie wspó³rzêdnych przeszkód
    przeszkoda_y = [przeszkoda_y; punkty(przeszkoda).Position.Y];
    
end

przeszkoda_x(1) = [];
przeszkoda_y(1) = [];
map = binaryOccupancyMap(10,10,10);
setOccupancy(map, [przeszkoda_x przeszkoda_y], ones(rozmiar(1)-2,1)); %dodanie przeszkód do mapy
inflate(map,1)
figure
show(map)

planner = mobileRobotPRM(map); %inicjalizacja funkcji planuj¹cej œcie¿kê
path = findpath(planner,robotInitialLocation,robotGoal); %utworzenie œcie¿ki

initialOrientation = yaw;
robotCurrentPose = [robotInitialLocation initialOrientation]';
controller = controllerPurePursuit; %inicjalizacja sterowania robota
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.2;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 1.5;
goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

while( distanceToGoal > goalRadius )
    [v, omega] = controller(robotCurrentPose); %obliczenie wartoœci prêdkoœci liniowej i k¹towej do sterowania robotem
    disp("v: " + v);
    disp("w: " + omega);
    odbierz = stany.receive;
    punkty = odbierz.Pose;
    robot_x = punkty(2).Position.X;
    robot_y = punkty(2).Position.Y;
    trasa_x = [trasa_x; robot_x];
    trasa_y = [trasa_y; robot_y];
    euler = quat2eul([punkty(2).Orientation.W punkty(2).Orientation.Z punkty(2).Orientation.Y punkty(2).Orientation.X]);
    yaw = euler(3);
    if(yaw<0)
        yaw = 2*pi+yaw;
    end
    robotCurrentPose = [robot_x; robot_y; yaw]; 
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    wiadomosc1.Angular.Z = omega; %wpisanie wartoœci do sterowania robota
    wiadomosc1.Linear.X = v;
    send(chodzenie, wiadomosc1); %wys³anie wartoœci sterowania do robota
    pause(0.1)
    hold off
    plot(path(:,1), path(:,2),"k--d")
    hold all
    plot(trasa_x, trasa_y, 'b-')
    show(map);
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "Parent", gca, "View","2D", "FrameSize", 2);
    light;
    xlim([0 13])
    ylim([0 13])
end

    wiadomosc1.Angular.Z = 0; %zatrzymanie robota
    wiadomosc1.Linear.X = 0;
    wiadomosc1.Linear.Y = 0;
    send(chodzenie, wiadomosc1);


