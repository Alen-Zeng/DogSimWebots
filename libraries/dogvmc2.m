%%
% syms O0 O1 O2 O3 O4 L L1 L2 Lt s
% L=0.2965; L1=0.33; L2=0.33;
% O0=pi/2; O1=-pi;O2=pi/2;
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
syms O0lf O1lf O2lf O3lf O4lf O0rf O1rf O2rf O3rf O4rf O0lb O1lb O2lb O3lb O4lb O0rb O1rb O2rb O3rb O4rb
syms L L1 L2 s
syms Ltlf Ltrf Ltlb Ltrb
syms Pl Pr Ql Qr Rl Sl Rr Sr Uf Wf Ub Wb
syms Jlf Jrf Jlb Jrb

L=0.2965; L1=0.33; L2=0.33; s=0.09;

O0lf=1.39633;O1lf=-2.44336;O2lf=1.04719;O3lf=0;O4lf=0;
O0rf=1.39633;O1rf=-2.44336;O2rf=1.04719;O3rf=0;O4rf=0;
O0lb=1.39633;O1lb=-2.44336;O2lb=1.04719;O3lb=0;O4lb=0;
O0rb=1.39633;O1rb=-2.44336;O2rb=1.04719;O3rb=0;O4rb=0;

Ltlf=L1*cos(O0lf) + L2*cos(O0lf+O1lf);
Ltrf=L1*cos(O0rf) + L2*cos(O0rf+O1rf);
Ltlb=L1*cos(O0lb) + L2*cos(O0lb+O1lb);
Ltrb=L1*cos(O0rb) + L2*cos(O0rb+O1rb);

Pl=-L1*cos(O0lf)-L2*cos(O0lf+O1lf)+L*sin(O0lf+O1lf+O2lf);
Pr=-L1*cos(O0rf)-L2*cos(O0rf+O1rf)+L*sin(O0rf+O1rf+O2rf);
Ql=-L1*sin(O0lf)-L2*sin(O0lf+O1lf)-L*cos(O0lf+O1lf+O2lf);
Qr=-L1*sin(O0rf)-L2*sin(O0rf+O1rf)-L*cos(O0rf+O1rf+O2rf);
Rl=-L1*cos(O0lb)-L2*cos(O0lb+O1lb)-L*sin(O0lb+O1lb+O2lb);
Rr=-L1*cos(O0rb)-L2*cos(O0rb+O1rb)-L*sin(O0rb+O1rb+O2rb);
Sl=-L1*sin(O0lb)-L2*sin(O0lb+O1lb)+L*cos(O0lb+O1lb+O2lb);
Sr=-L1*sin(O0rb)-L2*sin(O0rb+O1rb)+L*cos(O0rb+O1rb+O2rb);
Uf=s*sin(O3lf+O4lf)-Ltlf*cos(O3lf);
Ub=s*sin(O3lb+O4lb)-Ltlb*cos(O3lb);
Wf=-s*sin(O3rf+O4rf)-Ltrf*cos(O3rf);
Wb=-s*sin(O3rb+O4rb)-Ltrb*cos(O3rb);

% T=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0;
%    0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0;
%    0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0;
%    0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0;
%    0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
%    0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0;
%    0,0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,-1;
%    1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0,0,0;
%    Pl,0,Ql,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%    0,0,0,0,0,Pr,0,Qr,1,0,0,0,0,0,0,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,0,0,Rl,0,Sl,1,0,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,Rr,0,Sr,1,0;
%    0,Uf,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%    0,0,0,0,0,0,Wf,0,0,1,0,0,0,0,0,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,0,0,0,Ub,0,0,1,0,0,0,0,0;
%    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,Wb,0,0,1]

T=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0;
   0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0;
   0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0;
   0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0;
   0,0,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
   
   0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0;
   0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0;
   0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1;
   0,0,0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,0;
   0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0;
   0,0,0,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,0,0,0;
%    0,0,1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1,0,0;
   0,0,1,0,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0;

   Pl,0,Ql,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,Pr,0,Qr,1,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,Rl,0,Sl,1,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,Rr,0,Sr,1,0;
   0,Uf,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,Wf,0,0,1,0,0,0,0,0,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,Ub,0,0,1,0,0,0,0,0;
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,Wb,0,0,1]


% Det=det(T)
% Rank=rank(T)
% [B,p]=rref(T)
% T_=inv(T)
% T_=pinv(T)


%取出不为0的5列
% T_t=transpose(T_)

% K_t=[                                                       (Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                          0,   0,                                               -(Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                  0,                                                            -(Pl*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                         0,    0,                                               (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                   0,                                                            -(Pl*Pr*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                          0,    0,                                               (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                  0,                                                             (Pl*Pr*Rl)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                         0,    0,                                               -(Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                   0;
%                                                                                                                  0,                    -(Ub*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),   0,                                                                                                        0,                          (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                  0,                    (Ub*Uf*Wb)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),    0,                                                                                                       0,                          -(Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                  0,                     (Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),    0,                                                                                                       0,                         -(Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                  0,                   -(Ub*Uf*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),    0,                                                                                                        0,                           (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf);
%  (Pr*Ql*Rr - Pr*Ql*Rl + Ql*Rl*Rr + Pr*Rl*Sr + Pr*Rr*Sl + Qr*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0, 1/4, -(Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0, -(Pl*Qr*Rl - Pl*Qr*Rr + Pl*Rl*Sr + Pl*Rr*Sl + Ql*Rl*Rr + Qr*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,  1/4, (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0, -(Pl*Pr*Sl + Pl*Pr*Sr + Pl*Qr*Rr + Pr*Ql*Rr - Pl*Rr*Sl + Pr*Rr*Sl)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0,  1/4, (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0,  (Pl*Pr*Sl + Pl*Qr*Rl + Pr*Ql*Rl + Pl*Pr*Sr + Pl*Rl*Sr - Pr*Rl*Sr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,  1/4, -(Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0;
%                                                    (Pr*Rr + Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0,   0,                                   -(Pl*Pr*Rr + Pl*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0,                                                   -(Pl*Rl + Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,    0,                                   (Pl*Pr*Rl + Pr*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0,                                                   -(Pl*Pr + Pr*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0,    0,                                   (Pl*Pr*Rl + Pr*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0,                                                    (Pl*Pr + Pl*Rl)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,    0,                                   -(Pl*Pr*Rr + Pl*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0;
%                                                                                                                  0,           -(Ub*Wb + Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),   0,                                                                                                        0,              (Ub*Uf*Wb + Uf*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),                                                                                                                  0,           (Ub*Uf + Ub*Wb)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),    0,                                                                                                       0,              -(Ub*Uf*Wf + Ub*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),                                                                                                                  0,            (Uf*Wf + Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),    0,                                                                                                       0,             -(Ub*Uf*Wf + Ub*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),                                                                                                                  0,          -(Ub*Uf + Uf*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),    0,                                                                                                        0,               (Ub*Uf*Wb + Uf*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf))];
% 

K_t=[                                                        (Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                          0,   0,                                                 -(Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                  0,                                                              -(Pl*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                         0,    0,                                                 (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                   0,                                                                  -(Pl*Pr*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                          0,    0,                                                 (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                  0,                                                                  (Pl*Pr*Rl)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                         0,    0,                                                 -(Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                   0;
                                                                                                                  0,                    -(Ub*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),   0,                                                                                                          0,                          (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                    0,                    (Ub*Uf*Wb)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),    0,                                                                                                         0,                          -(Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                        0,                     (Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),    0,                                                                                                         0,                         -(Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                       0,                   -(Ub*Uf*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),    0,                                                                                                          0,                           (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf);
  (Pr*Ql*Rr - Pr*Ql*Rl + Ql*Rl*Rr + Pr*Rl*Sr + Pr*Rr*Sl + Qr*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0, 1/4,   -(Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0,   -(Pl*Qr*Rl - Pl*Qr*Rr + Pl*Rl*Sr + Pl*Rr*Sl + Ql*Rl*Rr + Qr*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,  1/4,   (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0,       -(Pl*Pr*Sl + Pl*Pr*Sr + Pl*Qr*Rr + Pr*Ql*Rr - Pl*Rr*Sl + Pr*Rr*Sl)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0,  1/4,   (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0,       (Pl*Pr*Sl + Pl*Qr*Rl + Pr*Ql*Rl + Pl*Pr*Sr + Pl*Rl*Sr - Pr*Rl*Sr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,  1/4,   -(Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0;
                                                    (Pr*Rr + Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0,   0,                                     -(Pl*Pr*Rr + Pl*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0,                                                     -(Pl*Rl + Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,    0,                                     (Pl*Pr*Rl + Pr*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0,                                                         -(Pl*Pr + Pr*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                          0,    0,                                     (Pl*Pr*Rl + Pr*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                  0,                                                         (Pl*Pr + Pl*Rl)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                         0,    0,                                     -(Pl*Pr*Rr + Pl*Rl*Rr)/(2*(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr)),                                                                                   0;
                                                                                                                  0,           -(Ub*Wb + Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),   0,                                                                                                          0,              (Ub*Uf*Wb + Uf*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),                                                                                                                    0,           (Ub*Uf + Ub*Wb)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),    0,                                                                                                         0,              -(Ub*Uf*Wf + Ub*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),                                                                                                                        0,            (Uf*Wf + Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),    0,                                                                                                         0,             -(Ub*Uf*Wf + Ub*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),                                                                                                                       0,          -(Ub*Uf + Uf*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf)),    0,                                                                                                          0,               (Ub*Uf*Wb + Uf*Wb*Wf)/(2*(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf))];


K=transpose(K_t);

Jlf=[-L1*cos(O0lf)-L2*cos(O0lf+O1lf)+L*sin(O0lf+O1lf+O2lf),0,-L1*sin(O0lf)-L2*sin(O0lf+O1lf)-L*cos(O0lf+O1lf+O2lf),1,0;
     -L2*cos(O0lf+O1lf)+L*sin(O0lf+O1lf+O2lf),0,-L2*sin(O0lf+O1lf)-L*cos(O0lf+O1lf+O2lf),1,0;
     L*sin(O0lf+O1lf+O2lf),0,-L*cos(O0lf+O1lf+O2lf),1,0;
     0,s*sin(O3lf+O4lf)-Ltlf*cos(O3lf),0,0,1;
     0,s*sin(O3lf+O4lf),0,0,1];

Jrf=[-L1*cos(O0rf)-L2*cos(O0rf+O1rf)+L*sin(O0rf+O1rf+O2rf),0,-L1*sin(O0rf)-L2*sin(O0rf+O1rf)-L*cos(O0rf+O1rf+O2rf),1,0;
     -L2*cos(O0rf+O1rf)+L*sin(O0rf+O1rf+O2rf),0,-L2*sin(O0rf+O1rf)-L*cos(O0rf+O1rf+O2rf),1,0;
     L*sin(O0rf+O1rf+O2rf),0,-L*cos(O0rf+O1rf+O2rf),1,0;
     0,-s*sin(O3rf+O4rf)-Ltrf*cos(O3rf),0,0,1;
     0,-s*sin(O3rf+O4rf),0,0,1];

Jlb=[-L1*cos(O0lb)-L2*cos(O0lb+O1lb)-L*sin(O0lb+O1lb+O2lb),0,-L1*sin(O0lb)-L2*sin(O0lb+O1lb)+L*cos(O0lb+O1lb+O2lb),1,0;
     -L2*cos(O0lb+O1lb)-L*sin(O0lb+O1lb+O2lb),0,-L2*sin(O0lb+O1lb)+L*cos(O0lb+O1lb+O2lb),1,0;
     -L*sin(O0lb+O1lb+O2lb),0,+L*cos(O0lb+O1lb+O2lb),1,0;
     0,s*sin(O3lb+O4lb)-Ltlb*cos(O3lb),0,0,1;
     0,s*sin(O3lb+O4lb),0,0,1];

Jrb=[-L1*cos(O0rb)-L2*cos(O0rb+O1rb)-L*sin(O0rb+O1rb+O2rb),0,-L1*sin(O0rb)-L2*sin(O0rb+O1rb)+L*cos(O0rb+O1rb+O2rb),1,0;
     -L2*cos(O0rb+O1rb)-L*sin(O0rb+O1rb+O2rb),0,-L2*sin(O0rb+O1rb)+L*cos(O0rb+O1rb+O2rb),1,0;
     -L*sin(O0rb+O1rb+O2rb),0,+L*cos(O0rb+O1rb+O2rb),1,0;
     0,-s*sin(O3rb+O4rb)-Ltrb*cos(O3rb),0,0,1;
     0,-s*sin(O3rb+O4rb),0,0,1];

% Jtotal=[
%      -L1*cos(O0lf)-L2*cos(O0lf+O1lf)+L*sin(O0lf+O1lf+O2lf),0,-L1*sin(O0lf)-L2*sin(O0lf+O1lf)-L*cos(O0lf+O1lf+O2lf),1,0, 0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0;
%      -L2*cos(O0lf+O1lf)+L*sin(O0lf+O1lf+O2lf),0,-L2*sin(O0lf+O1lf)-L*cos(O0lf+O1lf+O2lf),1,0,                           0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0;
%      L*sin(O0lf+O1lf+O2lf),0,-L*cos(O0lf+O1lf+O2lf),1,0,                                                                0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0;
%      0,s*sin(O3lf+O4lf)-Ltlf*cos(O3lf),0,0,1,                                                                           0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0;
%      0,s*sin(O3lf+O4lf),0,0,1,                                                                                          0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0;
%      
%      0,0,0,0,0, -L1*cos(O0rf)-L2*cos(O0rf+O1rf)+L*sin(O0rf+O1rf+O2rf),0,-L1*sin(O0rf)-L2*sin(O0rf+O1rf)-L*cos(O0rf+O1rf+O2rf),1,0,  0,0,0,0,0,0,0,0,0,0;
%      0,0,0,0,0, -L2*cos(O0rf+O1rf)+L*sin(O0rf+O1rf+O2rf),0,-L2*sin(O0rf+O1rf)-L*cos(O0rf+O1rf+O2rf),1,0,                            0,0,0,0,0,0,0,0,0,0;
%      0,0,0,0,0, L*sin(O0rf+O1rf+O2rf),0,-L*cos(O0rf+O1rf+O2rf),1,0,                                                                 0,0,0,0,0,0,0,0,0,0;
%      0,0,0,0,0, 0,-s*sin(O3rf+O4rf)-Ltrf*cos(O3rf),0,0,1,                                                                           0,0,0,0,0,0,0,0,0,0;
%      0,0,0,0,0, 0,-s*sin(O3rf+O4rf),0,0,1,                                                                                          0,0,0,0,0,0,0,0,0,0;
%      
%      0,0,0,0,0,0,0,0,0,0,   -L1*cos(O0lb)-L2*cos(O0lb+O1lb)-L*sin(O0lb+O1lb+O2lb),0,-L1*sin(O0lb)-L2*sin(O0lb+O1lb)+L*cos(O0lb+O1lb+O2lb),1,0,  0,0,0,0,0;
%      0,0,0,0,0,0,0,0,0,0,   -L2*cos(O0lb+O1lb)-L*sin(O0lb+O1lb+O2lb),0,-L2*sin(O0lb+O1lb)+L*cos(O0lb+O1lb+O2lb),1,0,                            0,0,0,0,0;
%      0,0,0,0,0,0,0,0,0,0,   -L*sin(O0lb+O1lb+O2lb),0,+L*cos(O0lb+O1lb+O2lb),1,0,                                                                0,0,0,0,0;
%      0,0,0,0,0,0,0,0,0,0,   0,s*sin(O3lb+O4lb)-Ltlb*cos(O3lb),0,0,1,                                                                            0,0,0,0,0;
%      0,0,0,0,0,0,0,0,0,0,   0,s*sin(O3lb+O4lb),0,0,1,                                                                                           0,0,0,0,0;
%      
%      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, -L1*cos(O0rb)-L2*cos(O0rb+O1rb)-L*sin(O0rb+O1rb+O2rb),0,-L1*sin(O0rb)-L2*sin(O0rb+O1rb)+L*cos(O0rb+O1rb+O2rb),1,0;
%      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, -L2*cos(O0rb+O1rb)-L*sin(O0rb+O1rb+O2rb),0,-L2*sin(O0rb+O1rb)+L*cos(O0rb+O1rb+O2rb),1,0;
%      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, -L*sin(O0rb+O1rb+O2rb),0,+L*cos(O0rb+O1rb+O2rb),1,0;
%      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,-s*sin(O3rb+O4rb)-Ltrb*cos(O3rb),0,0,1;
%      0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,-s*sin(O3rb+O4rb),0,0,1];
Jtotal=blkdiag(Jlf,Jrf,Jlb,Jrb);

% H=Jtotal*K
F=[0;
   0;
   1;
   0;
   0];
% tor=H*F
% jointtorlf=Jlf*F
% jointtorrf=Jrf*F
% jointtorlb=Jlb*F
% jointtorrb=Jrb*F


