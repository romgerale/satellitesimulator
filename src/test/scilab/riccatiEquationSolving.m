clear

A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];
B = [ 0 0 0; 0 0 0; 0 0 0; -0.0032 0 0; 0 -0.0028 0; 0 0 -0.0019];
C= [ 1 1 1 1 1 1];
D = [0 0 0];

R= eye(3);
Q = eye(6);

%A = [1 0 0 1 0 0; 1 0 0 0 1 0; 1 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];

%%M = [A -B*inv(R)*B';-Q -A'];

%%[U,L] = schur(M);

%%U11 = U(1:6,1:6);
%%U12 = U(1:6,7:12);

%%inv(U11);

%%P = mldivide(U11',U12');
%%P11 = U12'*(inv(U11)');

%%K = inv(R)*B'*P;
 
[PP1A,LL1A,KK1A,r1A] = care(A,B,Q,R);

%K1 = inv(R)*B'*P1;

%K2 = lqr(A,B,Q,R);

%%% mine
M = [A -B*inv(R)*B';-Q -A'];
[VKK,DKK] = eig(M);

%%% eig 2
VV12K = VKK(1:6,7:12);
VV22K = VKK(7:12,7:12);

PP22K = VV22K*inv(VV12K);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

AA = [3 -2;4 -1]; 
%%AA = [-3 2;1 1];
BB = [0;1];
CC = [1 -1];
QQ = eye(2);
RR = eye(1);
[PP,LL,KK,r] = care(AA,BB,QQ,RR);
%RR = 3;
%[PP,LL,KK] = care(AA,BB,CC'*CC,RR);

%%%
PP
KK
LL

%%% mine
MM = [AA -BB*inv(RR)*BB';-QQ -AA'];

%%% schur
[UU,LL1] = schur(MM);

UUT = UU';
UU11 = UUT(1:2,1:2);
UU12 = UUT(1:2,3:4);

PP1 = (UU12')*(inv(UU11)');
KK1 = inv(RR)*BB'*PP1;

%%% ordschur
[UUS,LL1S] = ordschur(UU,LL1,'lhp');

UUTS = UUS';
UU11S = UUTS(1:2,1:2);
UU12S = UUTS(1:2,3:4);

PP11 = (UU12S')*(inv(UU11S)');
KK11 = inv(RR)*BB'*PP11;

UU11S2 = UUS(1:2,1:2);
UU21S2 = UUS(3:4,1:2);
PP112 = (UU21S2)*(inv(UU11S2));
KK11 = inv(RR)*BB'*PP112;


%%
UU21 = UUT(3:4,1:2);
UU22 = UUT(3:4,3:4);

PP12 = (UU21')*(inv(UU22)');
KK12 = inv(RR)*BB'*PP12;


%%% eig
[VV,DD] = eig(MM);
VV11 = VV(1:2,1:2);
VV21 = VV(3:4,1:2);

PP2 = VV21*inv(VV11);
KK2 = inv(RR)*BB'*PP2;

%%% eig 2
VV12 = VV(1:2,3:4);
VV22 = VV(3:4,3:4);

PP22 = VV22*inv(VV12);
KK22 = inv(RR)*BB'*PP22;


% numerically poor
%             OK         OK(ordschur)                                                   OK(eigen)
S = [eig(AA) eig(AA-BB*KK) eig(AA-BB*KK11) eig(AA-BB*KK1) eig(AA-BB*KK12) eig(AA-BB*KK2) eig(AA-BB*KK22)]


%% checking
SS = [(AA'*PP + PP*AA - PP*BB*inv(RR)*BB'*PP + QQ) (AA'*PP1 + PP1*AA - PP1*BB*inv(RR)*BB'*PP1 + QQ) (AA'*PP11 + PP11*AA - PP11*BB*inv(RR)*BB'*PP11 + QQ) (AA'*PP2 + PP2*AA - PP2*BB*inv(RR)*BB'*PP2 + QQ) (AA'*PP22 + PP22*AA - PP2*BB*inv(RR)*BB'*PP22 + QQ)]

%% Kleinman algorithm

      %%from eigen
P1 = PP22;
K1 = -P1*BB*inv(RR);

aerror = 1;
epsilon = 1e-100;
i=1;
while aerror > epsilon && i< 100
    
    %%
    X = AA+BB*K1';
    Y = -K1*RR*K1' - QQ;
    
    X1=kron(X',eye(size(X))) + kron(eye(size(X)),X');
    Y1=reshape(Y,prod(size(Y)),1); %%stack
    
    PX = inv(X1)*Y1;
    P = reshape(PX,sqrt(length(PX)),sqrt(length(PX))); %%unstack
    
    aerror = norm(P - P1);
    
    P
    aerror
    i
    
    P1 = P;
    K1 = -P1*BB*inv(RR);
    i=i+1;
end

P1

%% checking
[(AA'*PP + PP*AA - PP*BB*inv(RR)*BB'*PP + QQ) (AA'*PP11 + PP11*AA - PP11*BB*inv(RR)*BB'*PP11 + QQ) (AA'*P1 + P1*AA - P1*BB*inv(RR)*BB'*P1 + QQ)]


%% Riccati Equations in Optimal Control Theory_VERY_IMPORTANT_RICATI
n = 2;

U = UU;
T = LL1;

% Make it a complex Schur form for easy eigenvalue swapping.
for j=1:2*n-1
    if (abs(T(j+1,j))>1e-16)
        [V,D]=eig(T(j:j+1,j:j+1));
        V
        D
        [G,r]=qr(V);
        G
        T(j:j+1,:)=G'*T(j:j+1,:);
        T(:,j:j+1)=T(:,j:j+1)*G;
        U(:,j:j+1)=U(:,j:j+1)*G;
    end
end

% Swap eigenvalues so that unstable eigenvalues are first
% To get a negative definite solution to the CARE.
% Comment out to get no sorting of eigenvalues.
% Currently this is set to give a positive definite solution
% to the Riccati equation.
for j=1:2*n
    for k=1:2*n-1
        if (real(T(k,k)) > real(T(k+1,k+1)))
            G=givens(T(k,k+1),T(k+1,k+1)-T(k,k));
            T(k:k+1,:)=G*T(k:k+1,:);
            T(:,k:k+1)=T(:,k:k+1)*G';
            U(:,k:k+1)=U(:,k:k+1)*G';
            T(k+1,k)=0;
        end
    end
end
% Form the maximal CARE solution.
G=U(1:2*n,1:n)/U(1:n,1:n);
P=G(n+1:2*n,:);
J=eye(2*n); J=J([[n+1:2*n],[1:n]],:);
J(1:n,n+1:2*n)=-J(1:n,n+1:2*n);