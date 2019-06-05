plot(XD(1:101,1),XD(1:101,2),'r','Linewidth',4) 
xlim([0,40]);
axis equal
hold on
plot(XD(101:301,1),XD(101:301,2),'b','Linewidth',2)
xlabel('[cm]','Linewidth',4);
ylabel('[cm]','Linewidth',4);
%tratto 2
plot(XD(1:101,1),XD(1:101,2),'b','Linewidth',2) 
xlim([0,40]);
axis equal
hold on
plot(XD(101:301,1),XD(101:301,2),'r','Linewidth',4)
xlabel('[cm]','Linewidth',4);
ylabel('[cm]','Linewidth',4);

% intera

plot(XD(:,1),XD(:,2),'b','Linewidth',4) 
xlim([0,40]);
axis equal
