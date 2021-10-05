function [figureHandle] = generateRobotFigure(planProb)
%GENERATEROBOTFIGURE Generates the main figure in which the robots will be
%plotted.
    figureHandle = figure();
    hold on;
    figureHandle.Name  = 'Robot_Workspace';
    figureHandle.Units = 'normalized';
    figureHandle.WindowState = 'maximized';
    figureHandle.NumberTitle = 'off';
    figureHandle.Position = [0,0.05,1,1];
    % Visualize the world.
    for ii = 1 : length(planProb.collWorld)
       visualize(planProb.collWorld{ii}{1}, planProb.collWorld{ii}{2});
    end
    % Set the view of the camera.
    camproj('orthographic');
    view(-90,90);
    axis square;
    xlim([-0.5 2.5]);
    ylim([-1.5 1.5]);
    zlim([-0.3 1.5]);
    xlabel('$X\ in\ [m]$','fontsize',14,'interpreter','latex')
    ylabel('$Y\ in\ [m]$','fontsize',14,'interpreter','latex')
    zlabel('$Z\ in\ [m]$','fontsize',14,'interpreter','latex')
end

function [] = visualize(object, rgba)
%VISUALIZE Visualize an object within the existing figure.
    [~, objectAxisHandle] = show(object);
    objectAxisHandle.FaceColor = [rgba(1) rgba(2) rgba(3)];
    objectAxisHandle.FaceAlpha = rgba(4);
end
