function [eta,nu,tau] = backstepping(r, dr, d2r,eta,nu,h,useOde45)

    %% VARIABLES
    pL = 0.7;
    pm_c = 2.5;
    pm_L = 2;
    pg = 9.81;
    pd = 0.01;
    epsilon = 0.001;


    addpath('generated/');
%     M = @(eta)(f_5dof_MassMatrix(eta, pL, pm_c, pm_L));
%     C = @(eta, nu)(f_5dof_CoreolisMatrix(eta, nu, pL, pm_c, pm_L));
    G = @(eta)(f_5dof_Gravity(eta, pL, pm_c, pm_L, pg));
    D = diag([0 0 0 pd pd]);

    % Restate for singularity avoidance
    M = @(eta)(f_5dof_MassMatrix_singularity_avoidance(eta, pL, pm_c, pm_L, epsilon));
    C = @(eta, nu)(f_5dof_CoreolisMatrix_singularity_avoidance(eta, nu, pL, pm_c, pm_L, epsilon));






    %%

   
    % Do some control
    tau = zeros(5,1); 
    Kp = 1;
    Kd = 10;

    % pos error
    e_pos = r - eta(1:3);
    % vel error
    e_vel = dr - nu(1:3);


    tau(1:3) = Kp*e_pos + Kd*e_vel;
    tau(3) = tau(3,1) - pg*(pm_L + pm_c);

%     if abs(tau(1)) > 10000
%         tau(1) = 0;
%         fprintf(' \n tau(1) too high! set to zero \n');
%     elseif abs(tau(2)) > 10000
%         tau(2) = 0;
%         fprintf(' \n tau(2) too high! set to zero \n');
%     elseif abs(tau(3)) > 10000
%         tau(3) = 0;
%         fprintf(' \n tau(3) too high! set to zero \n');
%     end
%     if tau(1) > 1000
%         tau(1) = 1000;
%     elseif tau(1) < -1000
%         tau(1) = -1000;
%     end
%     
%     if tau(2) > 1000
%         tau(2) = 1000;
%     elseif tau(2) < -1000
%         tau(2) = -1000;
%     end
%     if tau(3) < -1500
%         tau(3) = -1500;
%     elseif tau(3) > -3500
%         tau(3) = -3500;
%     end

 
  
   if useOde45

              % Integrate velocity
       f = @(nu, tau)(M(eta)\(tau - C(eta, nu)*nu - G(eta) - D*diag(abs(nu))*nu));
       dnu = f(nu, tau);
%        interf = @(t,x) f(nu,tau);
       interf = @(t,x) f(x, tau);
       [~, temp] = ode45(interf, 0:0.001:h, nu);
       nu_next = temp(end,:)';
        clear temp;
       
       % Simulate position integration
       f = @(eta, nu)(nu);
%        interf = @(t,x) f(eta,nu);
       interf = @(t,x) f(x, nu);
       [~, temp] = ode45(interf, 0:0.001:h,eta);
       eta_next = temp(end,:)';
       clear temp;
   

   else
       
                % Simulate position integration
       f = @(eta, nu)(nu);
       eta_next = rk4(f, eta, nu, h);
                % Integrate velocity
       f = @(nu, tau)(M(eta)\(tau - C(eta, nu)*nu - G(eta) - D*diag(abs(nu))*nu));
       dnu = f(nu, tau);
       nu_next  = rk4(f, nu, tau, h);
        

       

   end
    
   % Update current state
   eta = eta_next;
   nu  = nu_next; 
   
   if eta(4)>pi/2
       eta(4) = pi/2;
       fprintf(' \n eta(4) too high! set to 180 \n');
   elseif eta(4) < -pi/2;
       eta(4) = -pi/2;
       fprintf(' \n eta(4) too high! set to -180 \n');
   end
    if eta(5)>pi/2
       eta(5) = pi/2;
   elseif eta(5) < -pi/2
       eta(5) = -pi/2;
   end
end