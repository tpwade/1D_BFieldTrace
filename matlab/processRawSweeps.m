
%addpath('/home/twade/Documents/Work/gradients/U01/H12/fieldMaps');

sweepDir = '../rawSweepLogs/'

rSensor = 626/2+8;

fDeg = [0:5:355];
pbrDeg = [270:-5:0 355:-5:275];

%BeField = zeros(709*length(fDeg),6);
BeField = [];
GxField = [];
GyField = [];
GzField = [];

windowSize = 11;
b = (1/windowSize)*ones(windowSize,1);
a = 1;

zOffset = 835.3; %595.974+239.3;


for iDeg = 1:length(fDeg);
    BeFieldTmp = [];
    GxFieldTmp = [];
    GyFieldTmp = [];
    GzFieldTmp = [];

    %[iDeg fDeg(iDeg)]

    fName = sprintf('%sminicom_%03d.log',sweepDir,fDeg(iDeg));

    fid = fopen(fName,'r');

    [txtLine,txtEnd] = fgets(fid);

    while(~strcmp(txtLine(1:2),'Be'))
        %searching
        [txtLine,txtEnd] = fgets(fid);
    end
    %[txtLine,txtEnd] = fgets(fid);
    %inNum = sscanf(txtLine,'%f,%f,%f,%f,%f,%f')';
    inNum = [0,0,0,0,0,0];
    while ( inNum(3)<1201.5 )
        [txtLine,txtEnd] = fgets(fid);
        inNum = sscanf(txtLine,'%f,%f,%f,%f,%f,%f')';
        BeFieldTmp = [BeFieldTmp; rSensor, pbrDeg(iDeg) inNum(3) inNum(4:6)];
    end
    if (size(BeFieldTmp,1)~= 709)
        [iDeg fDeg(iDeg)]
    end
    BeFieldTmp(:,4) = filter(b,a,BeFieldTmp(:,4));
    BeFieldTmp(:,5) = filter(b,a,BeFieldTmp(:,5));
    BeFieldTmp(:,6) = filter(b,a,BeFieldTmp(:,6));

    BeField = [BeField; BeFieldTmp];




    [txtLine,txtEnd] = fgets(fid);
    while(~strcmp(txtLine(1),'X'))
        %searching
        [txtLine,txtEnd] = fgets(fid);
    end

    inNum = [0,0,0,0,0,0];
    while ( inNum(3)<1201.5 )
        [txtLine,txtEnd] = fgets(fid);
        inNum = sscanf(txtLine,'%f,%f,%f,%f,%f,%f')';
        GxFieldTmp = [GxFieldTmp; rSensor, pbrDeg(iDeg) inNum(3) inNum(4:6)];
    end
    if (size(GxFieldTmp,1)~= 709)
        [iDeg fDeg(iDeg)]
    end

    GxField = [GxField; GxFieldTmp];




    [txtLine,txtEnd] = fgets(fid);
    while(~strcmp(txtLine(1),'Y'))
        %searching
        [txtLine,txtEnd] = fgets(fid);
    end

    inNum = [0,0,0,0,0,0];
    while ( inNum(3)<1201.5 )
        [txtLine,txtEnd] = fgets(fid);
        inNum = sscanf(txtLine,'%f,%f,%f,%f,%f,%f')';
        GyFieldTmp = [GyFieldTmp; rSensor, pbrDeg(iDeg) inNum(3) inNum(4:6)];
    end
    if (size(GyFieldTmp,1)~= 709)
        [iDeg fDeg(iDeg)]
    end

    GyField = [GyField; GyFieldTmp];




    [txtLine,txtEnd] = fgets(fid);
    while(~strcmp(txtLine(1),'Z'))
        %searching
        [txtLine,txtEnd] = fgets(fid);
    end

    inNum = [0,0,0,0,0,0];
    while ( inNum(3)<1201.5 )
        [txtLine,txtEnd] = fgets(fid);
        inNum = sscanf(txtLine,'%f,%f,%f,%f,%f,%f')';
        GzFieldTmp = [GzFieldTmp; rSensor, pbrDeg(iDeg) inNum(3) inNum(4:6)];
    end
    if (size(GzFieldTmp,1)~= 709)
        [size(GzFieldTmp,1) iDeg fDeg(iDeg)]
    end

    GzField = [GzField; GzFieldTmp];



    fclose(fid);
end
BeField = sortrows(BeField,[2,3]);
GxField = sortrows(GxField,[2,3]);
GyField = sortrows(GyField,[2,3]);
GzField = sortrows(GzField,[2,3]);


for i = 1:4
    switch i
    case 1
        fid = fopen('BeBField.csv','w');
        bFieldOut = BeField;
        fprintf(fid,'16\nField: Be\nMatrix:709x72\n');
    case 2
        fid = fopen('GxBField.csv','w');
        bFieldOut = GxField;
        fprintf(fid,'16\nField: Gx\nMatrix:709x72\n');
    case 3
        fid = fopen('GyBField.csv','w');
        bFieldOut = GyField;
        fprintf(fid,'16\nField: Gy\nMatrix:709x72\n');
    case 4
        fid = fopen('GzBField.csv','w');
        bFieldOut = GzField;
        fprintf(fid,'16\nField: Gz\nMatrix:709x72\n');
    end

    fprintf(fid,' rSensor = %5.1f mm (approx)\n',rSensor);
    fprintf(fid,' z=0 in Log is at service end, approx z=%5.1f mm in PBR\n',zOffset);
    fprintf(fid,'   0 deg = Patient Left          (Y=0,X+  in PBR)\n');
    fprintf(fid,'  90 deg = Patient Back (bottom) (Y+ ,X=0 in PBR)\n');
    fprintf(fid,' 180 deg = Patient Right         (Y=0,X-  in PBR)\n');
    fprintf(fid,' 270 deg = Patient Nose (top)    (Y- ,X=0 in PBR)\n');
    fprintf(fid,' 270 deg = Patient Nose (top)    (Y- ,X=0 in PBR)\n');
    fprintf(fid,' Sensor bField:\n');
    fprintf(fid,'   Bx is tangential, positive CCW looking at patient end\n');
    fprintf(fid,'   By along z, positive toward the patient end\n');
    fprintf(fid,'   Bz radial outward\n');
    fprintf(fid,'   relative to PBR coordinates, BxBy are rotated 180 deg\n');
    fprintf(fid,' theta(deg), zSensor (mm), Bx, By, Bz (uT)\n');

    fprintf(fid,'%4.0f,%6.1f,%7.2f,%7.2f,%7.2f\n',bFieldOut(:,2:end)');

    fclose(fid);
end

return

GxField(:,4:6) = GxField(:,4:6) - BeField(:,4:6);
GyField(:,4:6) = GyField(:,4:6) - BeField(:,4:6);
GzField(:,4:6) = GzField(:,4:6) - BeField(:,4:6);
GxField(:,3) = zOffset-GxField(:,3);
GyField(:,3) = zOffset-GyField(:,3);
GzField(:,3) = zOffset-GzField(:,3);

temp = reshape(GzField(:,6),709,[]);
figH = figure; plot(GzField(1:709,3),temp(:,3:end-1),'-','Color',[0.5 0.5 0.5])

[zPts,zElem] = loadRoemer('z','model',2);
xyzLoc = [ones(709,1)*rSensor/1000 zeros(709,1) GzField(1:709,3)/1000];

bField = generateFieldMap(xyzLoc,zElem);
bField = bField*1e-7*1e6;

figure(figH)
hold on;
plot(GzField(1:709,3),bField(:,1)*(-20.2),'-','LineWidth',2,'Color',[1 0 0])
plot(GzField(1:709,3),temp(:,[end-1 1 2]),'-')

