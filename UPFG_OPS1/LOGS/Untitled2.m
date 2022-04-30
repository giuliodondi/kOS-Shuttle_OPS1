clear;clc;close all;
currentdir = pwd;
logn = 41;
path = strcat(currentdir,'\log',num2str(logn),'.txt');
A = dlmread(path);

alt=A(:,2);
dst=A(:,3);
stg=A(:,4);

E=zeros(length(alt),2);
O=zeros(length(alt),2);
T = zeros(851,2);

r=6371;

d=dst(end);

theta=180*d/(pi*r);

crd=2*r*sind(theta/2);


beta=0;
x=r*cosd(theta/2);

scale=1;

for i=1:length(alt)
    gamma=-theta/2 + beta;

    xp=x/cosd(gamma);
    xpp=r - xp;
    
    z=(xp + xpp)*sind(gamma);
    
    E(i,1) =z;
    E(i,2) = xpp*cosd(gamma);
    
    aa=alt(i)*scale;
    bb=192*scale;
    
    if ~(i>851)
        T(i,1) = z + aa*sind(gamma);
        T(i,2) = E(i,2) + aa*cosd(gamma);
    end
    O(i,1) = z + bb*sind(gamma);
    O(i,2) = E(i,2) + bb*cosd(gamma);
    
    
    
    if i<length(alt)
        beta=beta + 180*(dst(i+1)-dst(i))/(pi*r);
    end
end


cmap = jet(851); 

for i=1:length(cmap)
    if stg(i)==1 || stg(i)==2
        cmap(i,:)=[1, 0, 0];
    end
   if stg(i)==3 || stg(i)==4
        cmap(i,:)=[0, 0.5, 0];
   end
    if stg(i)==5
        cmap(i,:)=[0, 0, 1];
    end
    
end



figure
hold all
set(gca,'Color','k')
plot(E(:,1),E(:,2),'LineWidth',3);
plot(O(:,1),O(:,2),'w--','LineWidth',3);
scatter(T(:,1),T(:,2),10,cmap, 'filled');