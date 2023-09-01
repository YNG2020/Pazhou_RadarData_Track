function [maxVarX, maxVarY] = getMaxVarX_MaxVarY(maxCarX, maxCarY, theta2)
    theta = atan(maxCarY / maxCarX);
    maxCarLen = sqrt(maxCarX * maxCarX + maxCarY * maxCarY);
    maxVarX = abs(maxCarLen * cos(theta2 - theta));
    if maxVarX < abs(maxCarLen * cos(theta2 + theta))
        maxVarX = abs(maxCarLen * cos(theta2 + theta));
    end
    maxVarY = abs(maxCarLen * sin(theta2 - theta));
    if maxVarY < abs(maxCarLen * sin(theta2 + theta))
        maxVarY = abs(maxCarLen * sin(theta2 + theta));
    end
end