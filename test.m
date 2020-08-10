        % Winch/pulley locations (ai)
        FrameAnchors = [ ...
            [-4.70;  4.30] ...
          , [ 3.70;  3.20] ...
          , [ 3.50; -5.20] ...
          , [-4.50; -5.00] ...
        ];

        % Platform anchors (bi)
        %Platform = zeros(4,2);
        
        Platform = (20*[ -0.02 ,   0.02 ,  0.02 , -0.02 ;
                        -0.01 ,   -0.01 ,  0.01 , 0.01 ]);
                    
                            CableOffset = zeros(1,4);
                    
                    
                    
p = [0; 0; 1; 0; 0; 1];


        	r = [p(1); p(2)];
            
            R = [p(3), p(4); p(5), p(6)];
            
            
            
            L = FrameAnchors - ( repmat(r, 1, 4) + R * Platform );
            l = (sqrt( sum( ( L ).^2, 1)) + CableOffset);
            
            u = L ./ l;
            
            AT = vertcat( ...
                u ... % Force part
                , cross([R*Platform; zeros(1,4)], [u; zeros(1,4)]) ... % Torque part
                );
            
dot(R*Platform, [u(2,:); -u(1,:)]);



MassMatrix = 50; 
wrench = 10;

ddy = mldivide(MassMatrix, wrench)


