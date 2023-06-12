%orbit_bvp_how created by Geoff Huntington 2/21/07 %Solves the Hamiltonian Boundary Value Problem for the orbit-raising optimal %control problem (p.66 Bryson & Ho). Computes the solution using BVP4C %Invokes subroutines orbit_ivp and orbit_bound clear all;%close all; set(0, 'DefaultAxesFontSize', 14, 'DefaultAxesFontWeight','demi') set(0, 'DefaultTextFontSize', 14, 'DefaultTextFontWeight','demi')
%Fixed final time %Tf = 3.3155;
clc;clear; close all;
Tf = 4;
four=0; % not four means use bvp6c
%Constants
global mu m0 m1 T
mu=1; m0=1; m1=-0.07485; T= 0.1405;
%mu=1; m0=1; m1=-.2; T= 0.1405;
%Create initial Guess
n=100;
y = [ones(1,n); %r
zeros(1,n); %vr
ones(1,n); %vt
-ones(1,n); %lambda_r
-ones(1,n); %lambda_vr
-ones(1,n)]; %lambda_vt
x = linspace(0,Tf,n); %time
solinit.x = x;solinit.y = y;
%Set optimizer options
tol = 1E-10;
options = bvpset('RelTol',tol,'AbsTol',[tol tol tol tol tol tol],'Nmax', 2000);
%Solve
if four
sol = bvp4c(@orbit_ivp,@orbit_bound,solinit,options);
Nstep=40;
else
sol = bvp5c(@orbit_ivp,@orbit_bound,solinit,options);
Nstep=30;
end
%Plot results
figure(1);clf
plot(sol.x,sol.y(1:3,:),'LineWidth',2)
legend('r','v_r','v_t','Location','NorthWest')
grid on;
axis([0 4 0 2])
title('HBVP Solution')
xlabel('Time');ylabel('States')
figure(2);clf
plot(sol.x,sol.y(4:6,:),'LineWidth',2)
legend('p_1(t)','p_2(t)','p_3(t)','Location','NorthWest')
grid on;
axis([0 4 -3 2])
title('HBVP Solution')
xlabel('Time');ylabel('Costates')
ang2=atan2(sol.y(5,:),sol.y(6,:))+pi;
figure(3);clf
plot(sol.x,180/pi*ang2','LineWidth',2)
grid on;
axis([0 4 0 360])
title('HBVP Solution')
xlabel('Time');ylabel('Control input angle \phi(t)')
norm(tan(ang2')-(sol.y(5,:)./sol.y(6,:))')
print -f1 -dpng -r300 orbit1.png
print -f2 -dpng -r300 orbit2.png
print -f3 -dpng -r300 orbit3.png
% Code below adapted inpart from Bryson "Dynamic Optimization"
dt=diff(sol.x);
dth=(sol.y(3,1:end-1)./sol.y(1,1:end-1)).*dt; % \dot \theta = v_t/r
th=0+cumsum(dth');
pathloc=[sol.y(1,1:end-1)'.*cos(th) sol.y(1,1:end-1)'.*sin(th)];
figure(4);clf
plot(pathloc(:,1),pathloc(:,2),'k-','LineWidth',2)
hold on
zz=exp(sqrt(-1)*(0:.01:pi)');
r0=sol.y(1,1);rf=sol.y(1,end);
plot(r0*real(zz),r0*imag(zz),'r--','LineWidth',2)
plot(rf*real(zz),rf*imag(zz),'b--','LineWidth',2)
plot(r0,0,'ro','MarkerFace','r')
plot(rf*cos(th(end)),rf*sin(th(end)),'bo','MarkerFace','b')
fact=0.2;ep=ones(size(th,1),1)*pi/2+th-ang2(1:end-1)';
xt=pathloc(:,1)+fact*cos(ep); yt=pathloc(:,2)+fact*sin(ep);
for i=1:Nstep:size(th,1)
    pltarrow([pathloc(i,1);xt(i)],[pathloc(i,2);yt(i)],0.5,'m');
end
%axis([-1.6 1.6 -.1 1.8]);
axis([-2 2 -.1 1.8]);
axis('equal')
hold off
print -f4 -dpng -r300 orbit4.png;