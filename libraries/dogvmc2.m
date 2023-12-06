%%
% syms O0 O1 O2 O3 O4 L L1 L2 Lt s

% T4=[1,0,-L;
%     0,1,0;
%     0,0,1];
% T3=[cos(O2),-sin(O2),0;
%     sin(O2),cos(O2),L2;
%     0,0,1];
% T2=[cos(O1),-sin(O1),0;
%     sin(O1),cos(O1),L1;
%     0,0,1];
% T1=[cos(O0),-sin(O0),0;
%     sin(O0),cos(O0),0;
%     0,0,1];
% k=[0;
%    L2;
%    1];
% T=T1*T2*T3*T4
% q=T1*T2*k

% T4=[1,0,L;
%     0,1,0;
%     0,0,1];
% T3=[cos(O2),-sin(O2),0;
%     sin(O2),cos(O2),L2;
%     0,0,1];
% T2=[cos(O1),-sin(O1),0;
%     sin(O1),cos(O1),L1;
%     0,0,1];
% T1=[cos(O0),-sin(O0),0;
%     sin(O0),cos(O0),0;
%     0,0,1];
% T=T1*T2*T3*T4

% T1=[cos(O3),-sin(O3),0;
%     sin(O3),cos(O3),0;
%     0,0,1];
% T2=[cos(O4),-sin(O4),0;
%     sin(O4),cos(O4),Lt;
%     0,0,1];
% T3=[1,0,s;
%     0,1,0;
%     0,0,1];
% T= T1*T2*T3
%%
syms O0 O1 O2 O3 O4 L L1 L2 s Lt Pl Pr Ql Qr Rl Sl Rr Sr Uf Wf Ub Wb Jlf Jrf Jlb Jrb

% Lt=L1*cos(O0) + L2*cos(O0+O1);

% P=-L1*cos(O0)-L2*cos(O0+O1)+L*sin(O0+O1+O2);
% Q=-L1*sin(O0)-L2*sin(O0+O1)-L*cos(O0+O1+O2);
% R=-L1*cos(O0)-L2*cos(O0+O1)-L*sin(O0+O1+O2);
% S=-L1*sin(O0)-L2*sin(O0+O1)+L*cos(O0+O1+O2);
% U=s*sin(O3+O4)-Lt*cos(O3);
% W=-s*sin(O3+O4)-Lt*cos(O3);

T=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0;
   0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0;
   0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0;
   0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0;
   0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
   0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0;
   0,0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,-1;
   1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0,0,0;
   Pl,0,Ql,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,Pr,0,Qr,1,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,Rl,0,Sl,1,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,Rr,0,Sr,1,0;
   0,Uf,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,Wf,0,0,1,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,Ub,0,0,1,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,Wb,0,0,1];


% Det=det(T')
Rank=rank(T')
% [B,p]=rref(T)
T_=inv(T)


%取出不为0的5列
T_t=transpose(T_)
K_t=[               -(Ql*Qr*Rl*Sr + Ql*Qr*Rr*Sl)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                         0,                                              (Pl*Qr*Rl*Sr + Pl*Qr*Rr*Sl)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),   0,                                                                                   0,                   -(Ql*Qr*Rl*Sr + Ql*Qr*Rr*Sl)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                          0,                                          (Pr*Ql*Rl*Sr + Pr*Ql*Rr*Sl)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),    0,                                                                                  0,                    (Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                          0,                                         -(Pl*Qr*Rl*Sr + Pr*Ql*Rl*Sr)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),    0,                                                                                  0,                    (Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                         0,                                             -(Pl*Qr*Rr*Sl + Pr*Ql*Rr*Sl)/(2*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),    0,                                                                                   0;
                                                                                                         0,                    (Ub*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf),                                                                                                                                    0,   0,                          -(Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf),                                                                                                          0,                     (Ub*Uf*Wb)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf),                                                                                                                                0,    0,                         -(Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf),                                                                                                          0,                    -(Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf),                                                                                                                                0,    0,                          (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf),                                                                                                          0,                   -(Ub*Uf*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf),                                                                                                                                    0,    0,                           (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf);
                                    -(Ql*Qr*Sl*Sr)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),                                                                         0,                                                                (Pl*Qr*Sl*Sr)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),   0,                                                                                   0,                                     -(Ql*Qr*Sl*Sr)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),                                                                          0,                                                            (Pr*Ql*Sl*Sr)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),    0,                                                                                  0,                                      (Ql*Qr*Sl*Sr)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),                                                                          0,                                                           -(Ql*Qr*Rl*Sr)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),    0,                                                                                  0,                                      (Ql*Qr*Sl*Sr)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),                                                                         0,                                                               -(Ql*Qr*Rr*Sl)/(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl),    0,                                                                                   0;
  -(Ql*Qr*Sl + Ql*Qr*Sr + Ql*Sl*Sr + Qr*Sl*Sr)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                         0,        (Pl*Qr*Sl + Pl*Qr*Sr + Pl*Sl*Sr - Pr*Sl*Sr + Qr*Rl*Sr + Qr*Rr*Sl)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)), 1/4,                                                                                   0,   -(Ql*Qr*Sl + Ql*Qr*Sr + Ql*Sl*Sr + Qr*Sl*Sr)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                          0,    (Pr*Ql*Sl + Pr*Ql*Sr - Pl*Sl*Sr + Ql*Rl*Sr + Ql*Rr*Sl + Pr*Sl*Sr)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),  1/4,                                                                                  0,    (Ql*Qr*Sl + Ql*Qr*Sr + Ql*Sl*Sr + Qr*Sl*Sr)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                          0,   -(Ql*Qr*Rl + Pl*Qr*Sr + Pr*Ql*Sr - Ql*Qr*Rr + Ql*Rl*Sr + Qr*Rl*Sr)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),  1/4,                                                                                  0,    (Ql*Qr*Sl + Ql*Qr*Sr + Ql*Sl*Sr + Qr*Sl*Sr)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),                                                                         0,       -(Pl*Qr*Sl + Pr*Ql*Sl - Ql*Qr*Rl + Ql*Qr*Rr + Ql*Rr*Sl + Qr*Rr*Sl)/(4*(Pl*Qr*Sl*Sr + Pr*Ql*Sl*Sr - Ql*Qr*Rl*Sr - Ql*Qr*Rr*Sl)),  1/4,                                                                                   0;
                                                                                                         0,           (Ub*Wf + Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf)),                                                                                                                                    0,   0,              -(Ub*Uf*Wf + Uf*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf)),                                                                                                          0,            (Ub*Uf + Uf*Wb)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf)),                                                                                                                                0,    0,             -(Ub*Uf*Wf + Uf*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf)),                                                                                                          0,           -(Uf*Wb + Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf)),                                                                                                                                0,    0,              (Ub*Uf*Wb + Ub*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf)),                                                                                                          0,          -(Ub*Uf + Ub*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf)),                                                                                                                                    0,    0,               (Ub*Uf*Wb + Ub*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf + Ub*Wb*Wf - Uf*Wb*Wf))];
K=transpose(K_t)


Jlf=[-L1*cos(O0)-L2*cos(O0+O1)+L*sin(O0+O1+O2),0,-L1*sin(O0)-L2*sin(O0+O1)-L*cos(O0+O1+O2),1,0;
     -L2*cos(O0+O1)+L*sin(O0+O1+O2),0,-L2*sin(O0+O1)-L*cos(O0+O1+O2),1,0;
     L*sin(O0+O1+O2),0,-L*cos(O0+O1+O2),1,0;
     0,s*sin(O3+O4)-Lt*cos(O3),0,0,1;
     0,s*sin(O3+O4),0,0,1];

Jrf=[-L1*cos(O0)-L2*cos(O0+O1)+L*sin(O0+O1+O2),0,-L1*sin(O0)-L2*sin(O0+O1)-L*cos(O0+O1+O2),1,0;
     -L2*cos(O0+O1)+L*sin(O0+O1+O2),0,-L2*sin(O0+O1)-L*cos(O0+O1+O2),1,0;
     L*sin(O0+O1+O2),0,-L*cos(O0+O1+O2),1,0;
     0,-s*sin(O3+O4)-Lt*cos(O3),0,0,1;
     0,-s*sin(O3+O4),0,0,1];

Jlb=[-L1*cos(O0)-L2*cos(O0+O1)-L*sin(O0+O1+O2),0,-L1*sin(O0)-L2*sin(O0+O1)+L*cos(O0+O1+O2),1,0;
     -L2*cos(O0+O1)-L*sin(O0+O1+O2),0,-L2*sin(O0+O1)+L*cos(O0+O1+O2),1,0;
     -L*sin(O0+O1+O2),0,+L*cos(O0+O1+O2),1,0;
     0,s*sin(O3+O4)-Lt*cos(O3),0,0,1;
     0,s*sin(O3+O4),0,0,1];

Jrb=[-L1*cos(O0)-L2*cos(O0+O1)-L*sin(O0+O1+O2),0,-L1*sin(O0)-L2*sin(O0+O1)+L*cos(O0+O1+O2),1,0;
     -L2*cos(O0+O1)-L*sin(O0+O1+O2),0,-L2*sin(O0+O1)+L*cos(O0+O1+O2),1,0;
     -L*sin(O0+O1+O2),0,+L*cos(O0+O1+O2),1,0;
     0,-s*sin(O3+O4)-Lt*cos(O3),0,0,1;
     0,-s*sin(O3+O4),0,0,1];






