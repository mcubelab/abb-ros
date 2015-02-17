%First case is when we are just starting the program so the saver will %not exist. So we clear everything and initialize the params
grid =40;

if(~exist('saver'))
 
    close all;
    clc;
    
    rand('state',2000);
    %if(exist('tmp')) rmdir('tmp','s'); end
    mkdir tmp;
    disp('Starting Optimization');
    xlin = linspace(0,1,grid);
    ylin = linspace(0,1,grid);
    [X,Y] = meshgrid(xlin,ylin);
    nextP = [0;0]; % just initializing
    normP = [0;0];
    xTest = [X(:),Y(:)];
    saver = [];
    maxval = [];
    x = rand(nMin,2);
    y = zeros(nMin-1,1);
 
    i =1;   
    normP(1) = x(i,1);
    normP(2) = x(i,2);
    nextP(1)  = (x(i,1)*(MaxA1 - MinA1)) + MinA1;
    nextP(2) = (x(i,2)*(MaxA2 - MinA2)) + MinA2;
    i = i +1;
    
elseif(i > 1  && i <= nMin)
 
	disp('1 t0 nMin');
% Starting from 2 to nMin we collect samples
    %y= [y; userin];
    saver = [saver; normP(1) normP(2) userin];
    normP(1) = x(i,1);
    normP(2) = x(i,2);
    nextP(1)  = (x(i,1)*(MaxA1 - MinA1)) + MinA1;
    nextP(2) = (x(i,2)*(MaxA2 - MinA2)) + MinA2;
    i = i +1
 
elseif( i > nMin && i<= nMax )
    
    %Starting from nMin+1 to nMax we first save earlier case, then  
%get new set of params using GP and save them as nextP for Ros to read
 
 fprintf('\n nMin to nMax');
    %x = [x ; normP];
    %y=  [y; userin];
    saver = [saver; normP(1) normP(2) userin];
    
    covfunc = {'covSum',{'covSEiso','covNoise'}};
    %loghyper.cov = [log(0.15);log(0.30);log(0.35)];
    %loghyper.lik = [];
    %loghyper = [log(0.25); log(0.30); log(0.35)];
    %[fTest , s2] = gp(loghyper, @infExact, [], covfunc, [],saver(:,1:2) ,saver(:,3),xTest);
    loghyper = [log(0.15);log(0.30);log(0.35)];
    [fTest , s2] = gpr(loghyper, covfunc, saver(:,1:2) ,saver(:,3),xTest);
    s2 = s2 - exp(2*loghyper(3));
    
    F = 0*fTest;
    hold off;
    axis on;
    mesh(X,Y,reshape(F,grid,grid),'FaceColor','none','EdgeColor', [0.0 0.0 0.0]);
    axis equal;
    colormap(copper);
    hold on;
    mesh(X,Y,reshape(fTest,grid,grid),'FaceColor', 'flat','FaceAlpha',0.8);
    hold on;
    bound = fTest + 3*sqrt(s2);
    mesh(X,Y,reshape(bound,grid,grid),'FaceColor', 'flat','FaceAlpha',0.3, 'FaceColor','r');
    hold on
        
    %  Find max of function
    [val1 indMax1] = max(fTest);
    plot3(X(indMax1), Y(indMax1), val1,'.b','MarkerSize',16.0);
    %  Find max of bound to get next exploration
    maxval = [maxval ; i , X(indMax1), Y(indMax1) val1];
	
	disp(sprintf('i = %d, param1 = %f ,param2 = %f , prob = %f',i , X(indMax1), Y(indMax1),val1));
    [val2 indMax2] = max(bound);
    normP= [X(indMax2) , Y(indMax2)]
    plot3(normP(1), normP(2), val2,'.r','MarkerSize',16.0);
    axis off
    str = sprintf('nSamples = %d',i);
    text(-0.8, 0.5, 0, str);
    set(gcf,'PaperPositionMode','manual');
    set(gcf,'PaperUnits','inches');
    set(gcf,'PaperPosition',[0 0 3.2 2.4]);
    drawnow
 
    fName = sprintf('frame%.4d.jpg',i-nMin); 
    fName = fullfile('tmp',fName);
    saveas(gcf, fName);


    disp(['n=' num2str(i) ' out of ' num2str(nMax)]); 
    nextP(1)  = (normP(1)*(MaxA1 - MinA1)) + MinA1;
    nextP(2) = (normP(2)*(MaxA2 - MinA2)) + MinA2;
    i = i+1;
	if( 9 == rem(i,10))
		filename = ['run4iter' num2str(i)];
		save(filename);
	end
disp(sprintf('i = %d, param1 = %lf,param2 = %lf , prob = %lf',(i-1) , X(indMax1), Y(indMax1),val1));
elseif( i == nMax +1)
 
% this means program has run nMax times
 	fprintf('\n wrapping up');
    saver = [saver; normP(1) normP(2) userin];
    Param_optim = zeros(1,2);
    Param_optim(1) = (X(indMax1) *(MaxA1 - MinA1)) + MinA1;
    Param_optim(2) = (Y(indMax1) *(MaxA2 - MinA2)) + MinA2;
    Stoc_opt_generateVideo('tmp','frame','jpg',nMax-nMin+   1,'video.avi');
    movefile tmp/video.avi videoIEMax6.avi;
    disp('Finished');
 	save('run4final');
    save saver_completed.mat saver
 
end
 
 

