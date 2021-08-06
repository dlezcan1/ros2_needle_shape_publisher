function Rmat = FSframe(pmat,ds)
    %
    % compute the Frenet-Serret frame given the curve.
    % use for needle project.
    % convention: 
    %     - z-axis = tangent direction
    %     - x-axis = major curvature direction
    %
    % - written by Jin Seob Kim

    N = size(pmat,2);
    p0 = pmat(:,2) - 2*ds*[0;0;1];
    pmat_aug = [p0,pmat];

    % gradient, related to t vector
    gradv = gradient(pmat_aug,ds);

    % second derivative, Laplacian, related to n vector
    delv = gradient(gradv,ds);

    % frame
    Rmat = zeros(3,3,N);

    for i = 1:N
        % t vector
        tv = gradv(:,i+1)/norm(gradv(:,i+1),2);

    %     % n vector (orthogonalization)
    %     vv = (delv(:,i)'*tv)*tv;
    %     nv = delv(:,i) - vv;
    %     nv = nv/norm(nv,2);

        % or another way to compute n vector
        nv = cross(cross(tv,delv(:,i+1)),tv);
        nv = nv/norm(nv,2);

        % binormal vector
        bv = cross(tv,nv);

        % rotation matrix
        Rmat(:,:,i) = [nv, bv, tv];

        % test
        if abs(det(Rmat(:,:,i)) - 1) > 1e-8
            warning("Rmat(:,:,%d) is not SO(3)", i)
        end

    end

end
    