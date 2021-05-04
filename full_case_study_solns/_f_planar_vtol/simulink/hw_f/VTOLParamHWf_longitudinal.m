% Longitudinal transfer function (altitude)
Gh = tf(1/(P.mc+2*P.mr),[1 0 0]);

% Lead control design
Kh = 0.066;
numCh = [1/0.14 1];
denCh = [1/0.8 1];
Ch = tf(numCh,denCh);

% Closed-loop TF with lead in forward path
% Results in zero in CLTF
CLTF1 = minreal(Kh*Ch*Gh/(1+Kh*Ch*Gh));

% Closed-loop TF with lead zero in feedback path
% No zero in CLTF
Ch_n = tf([1/0.14 1],1);
Ch_d = tf(1,[1/0.8 1]);
CLTF2 = minreal(Kh*Ch_d*Gh/(1+Kh*Ch*Gh));

figure(1); clf;
rlocus(Ch*Gh);
axis([-1.5 0.1 -0.8 0.8]);
hold on;
rlocus(Ch*Gh,Kh,'b^');
sgrid(0.7,[0.275 10]);
hold off;

figure(2); clf;
step(CLTF1);
title('Step response with closed-loop zero');

figure(3); clf;
step(CLTF2);
title('Step response without closed-loop zero');

