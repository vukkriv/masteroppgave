function [eta,nu] = backstepping(r, dr, d2r,eta,nu,h)


    %% VARIABLES
    pL = 0.7;
    pm_c = 2.5;
    pm_L = 2;
    pg = 9.81;
    pd = 0.01;

    k1 = 1;
    k2 = 2;
    K1 = diag([k1 k1 k1]);
    K2 = diag([k2 k2 k2 k2 k2]);
    alpha_45 = [0; 0];

    addpath('generated/');
    M = @(eta)(f_5dof_MassMatrix(eta, pL, pm_c, pm_L));
    C = @(eta, nu)(f_5dof_CoreolisMatrix(eta, nu, pL, pm_c, pm_L));
    G = @(eta)(f_5dof_Gravity(eta, pL, pm_c, pm_L, pg));
    D = diag([0 0 0 pd pd]);
    H = [1 0 0 0 0;
         0 1 0 0 0;
         0 0 1 0 0];


    %%

    % Do some control
    tau = zeros(5,1); 
    tau(3) = - pg*(pm_L + pm_c);



    % Define error variables
    z1 = H*eta - r(:);

    % Virtual control 1-2
    alpha_13 = dr(:) - K1 * z1;
    alpha = [alpha_13; alpha_45];

    z2 = nu - alpha;

    dalpha_13 = d2r(:) - K1*H*z2 + K1*K1*z1;

    % Shorthands
    z45 = z2(4:5);
    theta_L = eta(5);
    phi_L = eta(4);

    G45 = G(eta);
    G45 = G45(4:5);

    K45 = K2(4:5,4:5);

    M4513 = M(eta);
    M4513 = M4513(4:5,1:3);

    Malpha = M(eta);
    Malpha = Malpha(4:5, 4:5);

    Calpha = C(eta, nu);
    Calpha = Calpha(4:5, 4:5);

    Dalpha = D(4:5, 4:5);

    % Define residual
    gamma = @(eta, pm_L)(-G45 + K45*z45 - M4513*dalpha_13);



    dalpha_45 = @(alpha_45, ~)( Malpha\(-Dalpha*alpha_45 - Calpha*alpha_45 + gamma(alpha_45, pm_L)));


    dalpha = [dalpha_13; dalpha_45(alpha_45, 0)];

    % Main control part
    tau = C(eta, nu)*alpha + D*alpha + G(eta) - H'*z1 + M(eta)*dalpha - K2*z2;

    % For fun, print third value
%     tau(3)

 % aand, set to zero
   tau(4:5) = [0; 0];
   

   
   % Integrate to get alpha_3
   alpha_45 = rk4(dalpha_45, alpha_45, 0, h);
   
   % Simulate position integration
   f = @(eta, nu)(nu);
   eta_next = rk4(f, eta, nu, h);
   
   % Integrate velocity
   f = @(nu, tau)(M(eta)\(tau - C(eta, nu)*nu - G(eta) - D*diag(abs(nu))*nu));
   dnu = f(nu, tau);
   nu_next  = rk4(f, nu, tau, h);
   
   % Update current state
   eta = eta_next;
   nu  = nu_next;
end