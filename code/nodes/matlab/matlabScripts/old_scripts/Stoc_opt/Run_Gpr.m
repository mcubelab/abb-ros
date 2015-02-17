if(1 == i)
    userin =0;
    normP = x(i,:)'
    nextP = normP .* (Param(:,2) - Param(:,1)) + Param(:,1);
    i = i +1;
    AllBest = [];

    
elseif(i > 1  && i <= nMin)
 
% Starting from 2 to nMin we collect samples
% Saving Step1 and setting up for Step2 and so on
    saver = [saver; normP' userin];
    MaxVal = [MaxVal; i-1 , nextP' , userin];
    normP = x(i,:)';
    nextP = normP .* (Param(:,2) - Param(:,1)) + Param(:,1)
    i = i +1
 
elseif( i > nMin && i<= nMax )
    
    %Starting from nMin+1 to nMax we first save earlier case, then  
%get new set of params using GP and save them as nextP for Ros to read
 
    
    saver = [saver; normP' userin];
    save CurrState.mat;
  
    MaxVal = [MaxVal; i-1 , nextP' ,userin];   
    covfunc = {'covSum',{'covSeiso','covNoise'}};
    loghyper = [log(0.15);log(0.30);log(0.35)];
    %loghyper = [log(0.25); log(0.30); log(0.35)];
    [fTest , s2] = gpr(loghyper, covfunc, saver(:,1:2),saver(:,3),xTest);
    
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


    maxP = [X(indMax1) ; Y(indMax1)];
    CurrBest_norm = [X(indMax1) ; Y(indMax1)];
    CurrBest = CurrBest_norm .* (Param(:,2) - Param(:,1)) + Param(:,1);
    AllBest = [AllBest;i-1 , CurrBest' , val1]

    %  Find max of bound to get next exploration
    [val2 indMax2] = max(bound);
    normP= [X(indMax2) ; Y(indMax2)];
  

   if(2 == nParams) 
    plot3(normP(1), normP(2), val2,'.r','MarkerSize',16.0);
    axis off
    str = sprintf('nSamples = %d',i);
    text(-0.8, 0.5, 0, str);
    set(gcf,'PaperPositionMode','manual');
    set(gcf,'PaperUnits','inches');
    set(gcf,'PaperPosition',[0 0 3.2 2.4]);
    drawnow
 
    fName = sprintf('frame%.4d.jpg',i-nMin+1); 
    fName = fullfile('tmp_1',fName);
    saveas(gcf, fName);
   end
    clc
    disp(['n=' num2str(i) ' out of ' num2str(nMax)]); 
    nextP = normP .* (Param(:,2) - Param(:,1)) + Param(:,1);
    i = i+1


elseif( i == nMax +1)
 
% this means program has run nMax times
 
    saver = [saver; normP' userin];
    MaxVal = [MaxVal; i-1 , nextP' ,userin];   

    CurrBest_norm = [X(indMax1) ; Y(indMax1)];
    Param_optim = CurrBest_norm .* (Param(:,2) - Param(:,1)) + Param(:,1);
   
    Stoc_opt_generateVideo('tmp_1','frame','jpg',nMax-nMin+   1,'video.avi');
    movefile tmp_1/video.avi videoIEMax6.avi;
    disp('Finished');
 
    save Parameters.mat Param_optim;
    save saver_completed.mat saver;
    save FinalState.mat;
 
end
 
 

