clc;
clear;
close all;
M = 1;
Ua=[];
Ub=[];
Uc=[];
Angle = 0:1:360;
for angle = Angle
   theta = angle;% + 330;
    theta = theta - round(theta/360)*360;
    if (theta<0) 
        theta = theta + 360;
    end
    if theta >= 300
        ua = 0.5+0.5*M*cosd(theta+30);
        ub = 0.5+0.5*M*sind(theta-60);
        uc = 0.5 - 0.5*sqrt(3)*M*sind(theta+30);
        
    elseif theta >= 240
        ua = 0.5+0.5*sqrt(3)*M*cosd(theta);
        ub = 0.5+0.5*M*sind(theta);
        uc = 0.5 - 0.5*M*sind(theta);
        
    elseif theta >= 180
        ua = 0.5+0.5*M*sind(theta+60);
        ub = 0.5-0.5*sqrt(3)*M*cosd(theta+60);
        uc = 0.5 - 0.5*M*sind(theta+60);
        
    elseif theta >= 120
         ua = 0.5+0.5*M*cosd(theta+30);
         ub = 0.5+0.5*M*sind(theta - 60);
         uc = 0.5-0.5*sqrt(3)*M*sind(theta+30);
        
    elseif theta >= 60
        ua = 0.5+0.5*sqrt(3)*M*cosd(theta);
        ub = 0.5+0.5*M*sind(theta);
        uc = 0.5-0.5*M*sind(theta);
    else
        ua = 0.5+0.5*M*sind(theta+60);
        ub = 0.5-0.5*sqrt(3)*M*cosd(theta+60);
        uc = 0.5-0.5*M*sind(60+theta);
    end
    Ua = [Ua, ua];
    Ub = [Ub, ub];
    Uc = [Uc, uc];    
end

figure (1)
subplot(2,1,1)
plot(Angle,Ua,'r'); hold on;
plot(Angle,Ub,'g'); 
plot(Angle,Uc,'b'); 
grid on;
hold off;

subplot(2,1,2)
plot(Angle,Ua-Ub,'r'); hold on;
plot(Angle,Ub-Uc,'g'); 
plot(Angle,Uc-Ua,'b'); 
grid on;
hold off;