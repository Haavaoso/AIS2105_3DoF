clear

L_1 = 4;
L_2 = 9.8;

L = 22.5;
h = L*sqrt(3.0/4.0);

gamma_deg = 0;
gamma = gamma_deg/180*pi;

theta_deg = 0;
theta = theta_deg/180*pi;
z = 10; % 7.3 - 15.2
z_values = linspace(7, 17, 100); % Variable z capped between 8 and 15
x = 1;

p1 = [-L/2; -h/3; 0; 1];
p2 = [L/2; -h/3; 0; 1];
p3 = [0; 2*h/3; 0; 1];

%% Z CODE
servoAnglesDegVisual = zeros(100,3);
for i = 1:100
    q = z_values(1,i);

    rota = [
    cos(gamma), sin(gamma)*cos(theta), sin(gamma)*cos(theta), 0;
    0, cos(theta), -sin(theta), 0;
    -sin(gamma), cos(gamma) * sin(theta), cos(gamma)*cos(theta), q;
    0, 0, 0, 1
    ];

    a = rota*p1;
    b = rota*p2;
    c = rota*p3;

    posVec = [a b c];

    servoAng = [0 0 0];
    
    for y = 1:3
        alfa = acos((L_2^2+L_1^2-x^2-posVec(3,y)^2)/(2*L_1*sqrt(x^2+posVec(3,y)^2)));
        beta = atan(posVec(3,y)/x);
        servoAnglesDegVisual(i,y) = (alfa-beta)*180/pi;
    end
end

%% GAMMA CODE
GservoAnglesDegVisual = zeros(100,3);
for i = 1:100
    gamma1 = linspace(-20,20,100)*pi/180;

    Grota = [
    cos(gamma1(1,i)), sin(gamma1(1,i))*cos(theta), sin(gamma1(1,i))*cos(theta), 0;
    0, cos(theta), -sin(theta), 0;
    -sin(gamma1(1,i)), cos(gamma1(1,i)) * sin(theta), cos(gamma1(1,i))*cos(theta), z;
    0, 0, 0, 1
    ];

    Ga = Grota*p1;
    Gb = Grota*p2;
    Gc = Grota*p3;

    GposVec = [Ga Gb Gc];
    
    for y = 1:3
        Galfa = acos((L_2^2+L_1^2-x^2-GposVec(3,y)^2)/(2*L_1*sqrt(x^2+GposVec(3,y)^2)));
        Gbeta = atan(GposVec(3,y)/x);
        GservoAnglesDegVisual(i,y) = (Galfa-Gbeta)*180/pi;
    end
end
%% THETA CODE
TservoAnglesDegVisual = zeros(100,3);
for i = 1:100
    theta1 = linspace(-12,12,100)*pi/180;

    Trota = [
    cos(gamma), sin(gamma)*cos(theta1(1,i)), sin(gamma)*cos(theta1(1,i)), 0;
    0, cos(theta1(1,i)), -sin(theta1(1,i)), 0;
    -sin(gamma), cos(gamma) * sin(theta1(1,i)), cos(gamma)*cos(theta1(1,i)), z;
    0, 0, 0, 1
    ];

    Ta = Trota*p1;
    Tb = Trota*p2;
    Tc = Trota*p3;

    TposVec = [Ta Tb Tc];
    
    for y = 1:3
        Talfa = acos((L_2^2+L_1^2-x^2-TposVec(3,y)^2)/(2*L_1*sqrt(x^2+TposVec(3,y)^2)));
        Tbeta = atan(TposVec(3,y)/x);
        TservoAnglesDegVisual(i,y) = (Talfa-Tbeta)*180/pi;
    end
end
%% THETA AND GAMMA
AservoAnglesDegVisual = zeros(100,3);
for i = 1:100
    gamma2 = linspace(-10,10,100)*pi/180;
    theta2 = linspace(-10,10,100)*pi/180;

    Arota = [
    cos(gamma2(1,i)), sin(gamma2(1,i))*cos(theta2(1,i)), sin(gamma2(1,i))*cos(theta2(1,i)), 0;
    0, cos(theta2(1,i)), -sin(theta2(1,i)), 0;
    -sin(gamma2(1,i)), cos(gamma2(1,i)) * sin(theta2(1,i)), cos(gamma2(1,i))*cos(theta2(1,i)), z;
    0, 0, 0, 1
    ];

    Aa = Arota*p1;
    Ab = Arota*p2;
    Ac = Arota*p3;

    AposVec = [Aa Ab Ac];
    
    for y = 1:3
        Aalfa = acos((L_2^2+L_1^2-x^2-AposVec(3,y)^2)/(2*L_1*sqrt(x^2+AposVec(3,y)^2)));
        Abeta = atan(AposVec(3,y)/x);
        AservoAnglesDegVisual(i,y) = (Aalfa-Abeta)*180/pi;
    end
end
%% Z VIZ
figure
plot(z_values, servoAnglesDegVisual(:, 1), 'r', 'LineWidth', 2);
hold on
plot(z_values, servoAnglesDegVisual(:, 2), 'g', 'LineWidth', 2);
plot(z_values, servoAnglesDegVisual(:, 3), 'b', 'LineWidth', 2);

% Add labels and legend
xlabel('z');
ylabel('Servo Angle (degrees)');
title('Servo Angles vs. z');
legend('Servo 1', 'Servo 2', 'Servo 3');
grid on;

%% GAMMA VIZ
figure;
plot(gamma1*180/pi, GservoAnglesDegVisual(:, 1), 'r', 'LineWidth', 2);
hold on
plot(gamma1*180/pi, GservoAnglesDegVisual(:, 2), 'g', 'LineWidth', 2);
plot(gamma1*180/pi, GservoAnglesDegVisual(:, 3), 'b', 'LineWidth', 2);

% Add labels and legend
xlabel('gamma');
ylabel('Servo Angle (degrees)');
title('Servo Angles vs. gamma');
legend('Servo 1', 'Servo 2', 'Servo 3');
grid on;
%% THETA VIZ
figure
plot(theta1*180/pi, TservoAnglesDegVisual(:, 1), 'r', 'LineWidth', 2);
hold on
plot(theta1*180/pi, TservoAnglesDegVisual(:, 2), 'g', 'LineWidth', 2);
plot(theta1*180/pi, TservoAnglesDegVisual(:, 3), 'b', 'LineWidth', 2);

xlabel('gamma');
ylabel('Servo Angle (degrees)');
title('Servo Angles vs. gamma');
legend('Servo 1', 'Servo 2', 'Servo 3');
grid on;

%% ALL VIZ
figure
plot(theta2*180/pi, AservoAnglesDegVisual(:, 1), 'r', 'LineWidth', 2);
hold on
plot(theta2*180/pi, AservoAnglesDegVisual(:, 2), 'g', 'LineWidth', 2);
plot(theta2*180/pi, AservoAnglesDegVisual(:, 3), 'b', 'LineWidth', 2);

xlabel('gamma and theta');
ylabel('Servo Angle (degrees)');
title('Servo Angles vs. gamma');
legend('Servo 1', 'Servo 2', 'Servo 3');
grid on;

