function [class_tracks] = only_class_new(i,class_bb_bbox,track_id_ry,New_Track_r,X_radar_mmWave,Y_radar_mmWave,Z_radar_mmWave,TM,Xmax_bbox,Ymax_bbox,Xmin_bbox,Ymin_bbox,class_tracks)
    X_rad=[];
    Y_rad=[];
    Z_rad=[];
    
    X_proj_yolo = [];
    Y_proj_yolo = [];
    
    Class_k_yolo = string;
    
    Frame_rad_1_r=[];

    Track_id_rad = unique(New_Track_r(i,find(~all(New_Track_r(i,:)==0,1))));
    
    for i_r=1:length(Track_id_rad)
        X_rad(i_r) = mean(X_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(i_r))));
        Y_rad(i_r) = mean(Y_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(i_r))));
        Z_rad(i_r) = mean(Z_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(i_r))));
    end
    
    if length(Track_id_rad)>0
    Frame_rad_1_r = [-Y_rad', -Z_rad', X_rad'];
    worldPoints_rad_1_r = [Frame_rad_1_r*1000 ones(size(Frame_rad_1_r,1),1)];
    projectedPoints_rad_1_r = worldPoints_rad_1_r*TM;
    X_proj_r = projectedPoints_rad_1_r(:,1)./projectedPoints_rad_1_r(:,3);
    Y_proj_r = projectedPoints_rad_1_r(:,2)./projectedPoints_rad_1_r(:,3);
    
    nanidx = find(isnan(X_proj_r));
    Track_id_rad(nanidx) = [];
    X_proj_r(nanidx) = [];
    Y_proj_r(nanidx) = [];
    end
    
    Track_id_yolo = find(nonzeros(Xmax_bbox(i,:)));
    
    if length(Track_id_yolo)>0
    X_proj_yolo(1:length(Track_id_yolo)) = (Xmax_bbox(i,Track_id_yolo)+Xmin_bbox(i,Track_id_yolo))/2;
    Y_proj_yolo(1:length(Track_id_yolo)) = (Ymax_bbox(i,Track_id_yolo)+Ymin_bbox(i,Track_id_yolo))/2;
    Class_k_yolo(1:length(Track_id_yolo)) = class_bb_bbox(i,Track_id_yolo);        
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    C_ry = [];
    temp = [];
    temp_id = 0;
    C_new = [];
    idx_ry=[];
    tempidx_ry = [];
    buff_ry=[];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if (~isempty(Track_id_rad) & ~isempty(Track_id_yolo))

            for k = 1:length(Track_id_rad)
                C_ry(k,:) = vecnorm([X_proj_r(k)-X_proj_yolo; Y_proj_r(k)-Y_proj_yolo],2,1)';
            end
            [M_ry,u_new_ry,u_old_ry] = matchpairs(C_ry,150);
            
            for ii=1:length(M_ry(:,1))
                track_id_ry = track_id_ry+1;
                X_ry_c(track_id_ry) = (X_proj_r(M_ry(ii,1))+X_proj_yolo(M_ry(ii,2)))/2;
                Y_ry_c(track_id_ry) = (Y_proj_r(M_ry(ii,1))+Y_proj_yolo(M_ry(ii,2)))/2;
                class_tracks(Track_id_rad(M_ry(ii,1)),i)=Class_k_yolo(M_ry(ii,2));
                %M_track_ry = [M_track_ry;track_id_ry];
                Comb_Track_r(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))))=track_id_ry;
            end

    end



end

