function p = plotPlanes(normals, fitted_normals)

    figure
    hold on
    for i=1:2%size(normals,1)
        [xx,yy] = ndgrid(1:2,1:2);
        normal = normals(i,:);
        z = (-normal(1)*xx - normal(2)*yy - normal(4))/normal(3);
        size(xx)
        size(yy)
        size(z)
        surf(xx,yy,z)
    end
    for i=3:3
        [yy,zz] = ndgrid(1:3,1:3);
        normal = normals(i,:);
        x = (-normal(2)*yy - normal(3)*zz - normal(4))/normal(1);
        size(xx)
        size(yy)
        size(z)
        surf(x,yy,zz)
    end

    for i=4:5
        [xx,zz] = ndgrid(1:2,1:2);
        normal = normals(i,:);
        y = (-normal(1)*xx - normal(3)*zz - normal(4))/normal(2);
        size(xx)
        size(yy)
        size(z)
        surf(xx,y,zz)
    end
    
    hold off    
    
    figure
    hold on
    for i=1:2%size(normals,1)
        [xx,yy] = ndgrid(1:2,1:2);
        normal = fitted_normals(i,:);
        z = (-normal(1)*xx - normal(2)*yy - normal(4))/normal(3);
        size(xx)
        size(yy)
        size(z)
        surf(xx,yy,z)
    end
    for i=4:4
        [yy,zz] = ndgrid(1:3,1:3);
        normal = fitted_normals(i,:);
        x = (-normal(2)*yy - normal(3)*zz - normal(4))/normal(1);
        size(xx)
        size(yy)
        size(z)
        surf(x,yy,zz)
    end

    for i=[3 5]
        [xx,zz] = ndgrid(1:2,1:2);
        normal = fitted_normals(i,:);
        y = (-normal(1)*xx - normal(3)*zz - normal(4))/normal(2);
        size(xx)
        size(yy)
        size(z)
        surf(xx,y,zz)
    end
    
    hold off

end