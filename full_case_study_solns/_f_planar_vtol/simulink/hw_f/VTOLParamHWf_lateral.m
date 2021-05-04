
% Lateral transfer function (tau to theta) - inner loop 
b0  = 1/(P.Jc+2*P.mr*P.d^2);
Gth = tf(b0,[1 0 0]);

% Inner loop design: wn > 2.75 rad/s, zeta = 0.7
numth = [1/1.6 1];
denth = [1/12 1];
Dth = tf(numth,denth);
Kth = 0.31;

figure(1); clf;
rlocus(Dth*Gth); hold on;
axis([-15 5 -10 10]);
sgrid(0.7,[2.75 50]);
rlocus(Dth*Gth,Kth,'r^'); hold off;

CLTF_th = feedback(Kth*Dth*Gth,1);    % transfer function from theta desired
                                  % to theta
figure(2); clf;
step(CLTF_th);

% Include full inner loop dynamics in transfer function for outer-loop
% design
b1 = -P.Fe/(P.mc+2*P.mr);
a1 = P.mu/(P.mc+2*P.mr);

G = tf(b1,[1 a1 0]);    % transfer function from theta to z
Gz = CLTF_th*G;         % transfer function from theta desired to z

% Outer loop design: wn >0.275, zeta = 0.7
numz = [1/0.18 1];
denz = [1/1 1];
Dz = tf(numz,denz);
Kz = -0.006;

figure(3); clf;
rlocus(-Dz*Gz); hold on;
axis(2*[-0.9 0.1 -0.5 0.5]);
% axis([-15 5 -10 10]);
sgrid(0.7,[0.275 50]);
rlocus(Dz*Gz,Kz,'r^'); hold off;

CLTF_z = feedback(Kz*Dz*Gz,1);    % transfer function from z desired to z

figure(4); clf;
step(CLTF_z);

