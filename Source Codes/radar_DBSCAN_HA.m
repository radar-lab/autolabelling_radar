function [X_c_r,Y_c_r,Z_c_r,idx_img_r,M_track_r,track_id_r,New_Track_r,new_id_r,idx_r] = radar_DBSCAN_HA(TS1,i,TM,X_radar_mmWave,Y_radar_mmWave,Z_radar_mmWave,epsilon,MinPts,track_id_r,M_track_r,X_c_r,Y_c_r,Z_c_r,New_Track_r,new_id_r1)
    idx_r=[];
    new_id_r=[];
    Frame_rad_1_r = [-Y_radar_mmWave; -Z_radar_mmWave; X_radar_mmWave]';
    worldPoints_rad_1_r = [Frame_rad_1_r*1000 ones(size(Frame_rad_1_r,1),1)];
    projectedPoints_rad_1_r = worldPoints_rad_1_r*TM;
    X_proj_r = projectedPoints_rad_1_r(:,1)./projectedPoints_rad_1_r(:,3);
    Y_proj_r = projectedPoints_rad_1_r(:,2)./projectedPoints_rad_1_r(:,3);
    
    if (i>1&&(TS1(i)-TS1(i-1)>=2.5))
           M_track_r=[];
           X_c_r = [];
           Y_c_r = [];
           Z_c_r = [];
    end
    
    idx_img_r =intersect(find(X_proj_r(X_proj_r<=480)),find(Y_proj_r(Y_proj_r<=640)));
    %idx_img =intersect(find(X_proj(X_proj>0)),find(Y_proj(Y_proj>0)));

    if length(idx_img_r)>0

    idx_r = dbscan([X_radar_mmWave(idx_img_r)',Y_radar_mmWave(idx_img_r)',Z_radar_mmWave(idx_img_r)'],epsilon,MinPts);
    tr_t_r = length(unique(idx_r(find(idx_r~=-1))));
    if tr_t_r>0
        
        if i==1|isempty(M_track_r)
            new_id_r = 1;


            for j=1:tr_t_r
                track_id_r = track_id_r+1;
                X_c_r(new_id_r) = mean(X_radar_mmWave(find(idx_r==j)));
                Y_c_r(new_id_r) = mean(Y_radar_mmWave(find(idx_r==j)));
                Z_c_r(new_id_r) = mean(Z_radar_mmWave(find(idx_r==j)));
                New_Track_r(i,find(idx_r==j)) = track_id_r;
                M_track_r = [M_track_r;track_id_r];
                new_id_r = new_id_r+1;
            end
            
        else
            temp_r = [];

            for j=1:tr_t_r
                X_c1_r(j) = mean(X_radar_mmWave(find(idx_r==j)));
                Y_c1_r(j) = mean(Y_radar_mmWave(find(idx_r==j)));
                Z_c1_r(j) = mean(Z_radar_mmWave(find(idx_r==j)));           
            end
            C_r = [];
            for k = 1:tr_t_r
                C_r(k,:) = vecnorm([X_c1_r(k)-X_c_r; Y_c1_r(k)-Y_c_r; Z_c1_r(k)-Z_c_r],2,1)';
            end

            X_c_r = [];
            Y_c_r = [];
            Z_c_r = [];
            new_id_r = 1;
            [M_r,u_new_r,uold_r] = matchpairs(C_r,1);

            for kk = 1:length(M_r(:,1))
                X_c_r(new_id_r) = mean(X_radar_mmWave(find(idx_r==M_r(kk,1))));
                Y_c_r(new_id_r) = mean(Y_radar_mmWave(find(idx_r==M_r(kk,1))));
                Z_c_r(new_id_r) = mean(Z_radar_mmWave(find(idx_r==M_r(kk,1))));
                new_id_r = new_id_r+1;
                New_Track_r(i,find(idx_r==M_r(kk,1)))=M_track_r(M_r(kk,2));
                idx_r(find(idx_r==M_r(kk,1))) = M_track_r(M_r(kk,2))*ones(size(find(idx_r==M_r(kk,1))));
                
                temp_r=[temp_r;M_track_r(M_r(kk,2))];
            end

            for kkk = 1:length(u_new_r)
                X_c_r(new_id_r) = mean(X_radar_mmWave(find(idx_r==u_new_r(kkk))));
                Y_c_r(new_id_r) = mean(Y_radar_mmWave(find(idx_r==u_new_r(kkk))));
                Z_c_r(new_id_r) = mean(Z_radar_mmWave(find(idx_r==u_new_r(kkk))));
                new_id_r = new_id_r+1;
                track_id_r = track_id_r+1;
                New_Track_r(i,find(idx_r==u_new_r(kkk)))=track_id_r;
                idx_r(find(idx_r==u_new_r(kkk))) = track_id_r*ones(size(find(idx_r==u_new_r(kkk))));
                
                temp_r=[temp_r;track_id_r];
            end
            M_track_r = [];
            M_track_r = temp_r;       

        end   
    else
        new_id_r=new_id_r1;
    end
    else
        new_id_r=new_id_r1;
    end
end

