
clear all,clc
a=10;b=5;%parameters
Fo=-1;%force acting @hinge 2
TOL=1e-6;%Tolerance of Delta_v(iterative displacements)
E=10;%Youngs modulus of the truss member
A=1;%Cross sectional area of the truss member
NR_ARC=1;%NR_ARC=1-->Newton Raphson;2-->Arc Length
plot_routine_single_truss(a,b,0,1,'r');%plotting undisplaced configuration
OBJ1=VideoWriter('Movie_truss_1_NR','MPEG-4');
OBJ2=VideoWriter('Movie_truss_1_FD','MPEG-4');
open(OBJ1);
open(OBJ2);
del_lamda=0.01;%force increments
v=0;%initial displacement
lamda=0;%initial force increment
k_inc=0;
for lamda=del_lamda:del_lamda:1 %loop for force increments
    fprintf('Force increment; %3.2f\n',lamda);
    k_inc=k_inc+1;
    k_iter=0;
    converge=123456;
    while(converge>TOL)%NR iterations
        [Fi,K]=single_truss(a,b,E,A,v)%int. forces, TSM
        g=Fi-lamda*Fo;%force residual
        delta_v=-inv(K)*g;%increment in vertical displacement
        converge=abs(delta_v);%convergence measure
        v=v+delta_v;%current displacement update
        %uncomment to see displaced configurations within iterations
        %plot_routine_single_truss(a,b,v,1,'b');
        k_iter=k_iter+1;
        fprintf('iter:%3d disp: %3.4f f_res %3.4f \n',k_iter,v,g);
    end
    vstore(k_inc)=v;
    Fstore(k_inc)=lamda;
    fprintf('-----------------------------------------------\n');
    plot_routine_single_truss(a,b,v,1,'r');%plotting displ. configurations
    axis([-2 12 -10 10]);
    figure(1);
    currFrame1=getframe(gcf);
    writeVideo(OBJ1,currFrame1);
    pause(1e-6);
    figure(2);
    plot(vstore,abs(Fo)*Fstore,'b.','LineWidth',2);
    xlabel('v (displacement)','FontSize',16);
    ylabel('|F| (input force)','FontSize',16);
    axis([-15 0 0 1]);
    currFrame2=getframe(gcf);
    writeVideo(OBJ2,currFrame2);
    pause(1e-6);
end
close(OBJ1);
close(OBJ2);
