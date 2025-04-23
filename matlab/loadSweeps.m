
%addpath('/home/twade/Documents/Work/gradients/U01/H12/fieldMaps');

rSensor = 321;
zOffset = 835.3;

BeField = readmatrix('BeBField.csv');
GxField = readmatrix('GxBField.csv');
GyField = readmatrix('GyBField.csv');
GzField = readmatrix('GzBField.csv');

BeField = [rSensor*ones(size(BeField,1),1), BeField];
GxField = [rSensor*ones(size(GxField,1),1), GxField];
GyField = [rSensor*ones(size(GyField,1),1), GyField];
GzField = [rSensor*ones(size(GzField,1),1), GzField];


GxField(:,4:6) = GxField(:,4:6) - BeField(:,4:6);
GyField(:,4:6) = GyField(:,4:6) - BeField(:,4:6);
GzField(:,4:6) = GzField(:,4:6) - BeField(:,4:6);

GxField(:,3) = zOffset-GxField(:,3);
GyField(:,3) = zOffset-GyField(:,3);
GzField(:,3) = zOffset-GzField(:,3);


%%% Z %%%%%%%%%%%%

if 1==0

    [zPts,zElem] = loadRoemer('z','model',2);
    xyzLoc = [ones(709,1)*rSensor/1000 zeros(709,1) GzField(1:709,3)/1000];
    bField = generateFieldMap(xyzLoc,zElem);
    bField = bField*1e-7*1e6;



    temp = reshape( GzField(:,6),709,[])/20.2;
    figH = figure; plot(GzField(1:709,3),temp(:,[1:53 57:72]),'-','Color',[0.5 0.5 0.5])

    figure(figH)
    hold on;
    plot(GzField(1:709,3),bField(:,1)*(-1),'-','LineWidth',1.5,'Color',[1 0 0])
    plot(GzField(1:709,3),temp(:,[54:56]),'-')
    xlabel('z (mm)');
    ylabel('Br (uT/A)');
    title('Z Gradient, Br component')
    %https://www.mathworks.com/matlabcentral/answers/1626265-create-a-custom-legend
    qw{1} = plot(nan, '-','LineWidth',1.5,'Color',[1 0 0]);
    qw{2} = plot(nan, '-','Color',[0 0.4470 0.7410]);
    qw{3} = plot(nan, '-','Color',[0.8500 0.3250 0.0980]);
    qw{4} = plot(nan, '-','Color',[0.9290 0.6940 0.1250]);
    qw{5} = plot(nan, '-','Color',[0.5 0.5 0.5]);
    legend([qw{:}], {'Model','265','270','275','other'});
    xlim([-400 850])
    set(gcf,'units','centimeters');
    tempPos= get(gcf,'position');
    set(gcf,'position',[tempPos(1:2) 24 18]);
    print('-dpng','-r300','GzBr.png')






    temp = reshape(-GzField(:,5),709,[])/20.2;
    figH = figure; plot(GzField(1:709,3),temp(:,[1:53 57:72]),'-','Color',[0.5 0.5 0.5])

    figure(figH)
    hold on;
    plot(GzField(1:709,3),bField(:,3)*(-1),'-','LineWidth',1.5,'Color',[1 0 0])
    plot(GzField(1:709,3),temp(:,[54:56]),'-')
    xlabel('z (mm)');
    ylabel('Bz (uT/A)');
    title('Z Gradient, Bz component')
    %https://www.mathworks.com/matlabcentral/answers/1626265-create-a-custom-legend
    qw{1} = plot(nan, '-','LineWidth',1.5,'Color',[1 0 0]);
    qw{2} = plot(nan, '-','Color',[0 0.4470 0.7410]);
    qw{3} = plot(nan, '-','Color',[0.8500 0.3250 0.0980]);
    qw{4} = plot(nan, '-','Color',[0.9290 0.6940 0.1250]);
    qw{5} = plot(nan, '-','Color',[0.5 0.5 0.5]);
    legend([qw{:}], {'Model','265','270','275','other'});
    xlim([-400 850])
    set(gcf,'units','centimeters');
    tempPos= get(gcf,'position');
    set(gcf,'position',[tempPos(1:2) 24 18]);
    print('-dpng','-r300','GzBz.png');





    temp = reshape(-GzField(:,4),709,[])/20.2;
    figH = figure; plot(GzField(1:709,3),temp(:,[1:53 57:72]),'-','Color',[0.5 0.5 0.5])

    figure(figH)
    hold on;
    plot(GzField(1:709,3),bField(:,2)*(-1),'-','LineWidth',1.5,'Color',[1 0 0])
    plot(GzField(1:709,3),temp(:,[54:56]),'-')
    xlabel('z (mm)');
    ylabel('Bt (uT/A)');
    title('Z Gradient, Bt component')
    %https://www.mathworks.com/matlabcentral/answers/1626265-create-a-custom-legend
    qw{1} = plot(nan, '-','LineWidth',1.5,'Color',[1 0 0]);
    qw{2} = plot(nan, '-','Color',[0 0.4470 0.7410]);
    qw{3} = plot(nan, '-','Color',[0.8500 0.3250 0.0980]);
    qw{4} = plot(nan, '-','Color',[0.9290 0.6940 0.1250]);
    qw{5} = plot(nan, '-','Color',[0.5 0.5 0.5]);
    legend([qw{:}], {'Model','265','270','275','other'});
    xlim([-400 850])
    set(gcf,'units','centimeters');
    tempPos= get(gcf,'position');
    set(gcf,'position',[tempPos(1:2) 24 18]);
    print('-dpng','-r300','GzBt.png');

end





%%% x %%%%%%%%%%%%



[xPts,xElem] = loadRoemer('x','model',2);
xElem = xElem(sum(xElem(:,9)==[1,68],2)==0,:); % loops 1 & 68 were dropped in the winding
%[x,y,z] = pol2cart(GxField(1:19*709,1)/1000,GxField(1:19*709,2)*180/pi,GxField(1:19*709,3)/1000);
[x,y,z] = pol2cart(GxField(:,2)*pi/180,GxField(:,1)/1000,GxField(:,3)/1000);
rDot = [x,y]./(x.^2+y.^2).^(0.5);
xyzLoc = [x,y,z];
clear x y z
%xyzLoc = xyzLoc(1:5:end,:);

% roughly 5 minute calculation
%tic;bField = generateFieldMap(xyzLoc,xElem);toc
%bField = bField*1e-7*1e6;

temp = reshape( -GxField(:,5),709,[])/20.2; % z component
temp = temp(:,1:19); %0 thru 90
bFieldTemp = reshape(bField(:,3),709,[]);
bFieldTemp = bFieldTemp(:,1:19); %0 thru 90
figH = figure;
axH = plot(GzField(1:709,3),bFieldTemp(:,:),'-');
colorSet = parula(19);
colororder(figH,colorSet);
hold on
plot(GxField(1:709,3),temp(:,:),'-')
legend(axH',sprintfc('%3.0f',[0:5:90]))
xlim([-400 850])
ylabel('Bz')

temp = reshape( -GxField(:,5),709,[])/20.2; % z component
temp = temp(:,19:37); %90 thru 180
bFieldTemp = reshape(bField(:,3),709,[]);
bFieldTemp = bFieldTemp(:,19:37); %90 through 180
figH = figure;
axH = plot(GzField(1:709,3),bFieldTemp(:,:),'-');
colorSet = flipud(parula(19));
colororder(figH,colorSet);
hold on
plot(GxField(1:709,3),temp(:,:),'-')
legend(axH',sprintfc('%3.0f',[90:5:180]))
xlim([-400 850])
ylabel('Bz')
%


