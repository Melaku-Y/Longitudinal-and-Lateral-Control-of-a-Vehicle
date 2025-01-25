function y = Kinematic_Model(Psi, Vx, Beta)
    Vy = Beta*Vx;
    V = sqrt(Vx^2 + Vy^2);
    
    Xdot = V*cos(Psi+Beta);
    Ydot = V*sin(Psi+Beta);
    
    y = [Xdot, Ydot];
end