function [c_idx,p_idx,b_idx,m_idx,X_r_d,Y_r_d,Z_r_d,Dop_r_d,SNR_r_d,Az_r_d,El_r_d,TS_r_d,I_yolo_d,Label,Idx] = gen_rad_cam_data(bagfile,c_idx,p_idx,m_idx,b_idx,X_r_d,Y_r_d,Z_r_d,Dop_r_d,SNR_r_d,Az_r_d,El_r_d,TS_r_d,I_yolo_d,Label,Idx)

    %%

    u_idx = 1;


    %% 
    format long
    bag = rosbag(bagfile);
    %%
    sel = select(bag,'Topic','/usb_webcam/image_rect_color/compressed');
    image_data_rect = readMessages(sel);

    sel = select(bag,'Topic','/darknet_ros/bounding_boxes');
    bbox_data_yolo = readMessages(sel);

    sel = select(bag,'Topic','/sony_radar/radar_scan');
    radar_data = readMessages(sel);

    %%
    [TS1,n_frames_mmWave,X_radar_mmWave,Y_radar_mmWave,Z_radar_mmWave,SNR_radar_mmWave,TgtIdx_radar_mmWave,Dop_radar_mmWave,R_radar_mmWave,Vx_radar_mmWave,Vy_radar_mmWave,Vz_radar_mmWave,Az_radar_mmWave,El_radar_mmWave,TS_radar_mmWave] = extract_radar(radar_data);

    %% %%%%%%%%%%%%%% Extract Image Timestamps for Frame Association stage %%%%%%%%%%%% %%
    for i = 1:length(image_data_rect)
         Img_Ts_rect(i) = image_data_rect{i}.Header.Stamp.Sec+image_data_rect{i}.Header.Stamp.Nsec*1e-9;
    end

    for i = 1:length(bbox_data_yolo)
         bbox_Ts_yolo(i) = bbox_data_yolo{i}.Header.Stamp.Sec+bbox_data_yolo{i}.Header.Stamp.Nsec*1e-9;
         Img_bbox_Ts_yolo(i) = bbox_data_yolo{i}.ImageHeader.Stamp.Sec+bbox_data_yolo{i}.ImageHeader.Stamp.Nsec*1e-9;
    end

    %%
    ids_p_frm = zeros(length(image_data_rect),10);
    for i=1:length(image_data_rect)-1
        ids_p_frm(i,1:length(find((Img_bbox_Ts_yolo>=Img_Ts_rect(i))&(Img_bbox_Ts_yolo<Img_Ts_rect(i+1)))))=find((Img_bbox_Ts_yolo>=Img_Ts_rect(i))&(Img_bbox_Ts_yolo<Img_Ts_rect(i+1))); 
    end

    %% %%%%%%%%%%%%%% Associate Radar and Image Frames based on Timestamps %%%%%%%%%%%% %%
    for i = 1:n_frames_mmWave
        comp_rect = ones(size(Img_Ts_rect))*(TS1(i));
        diff_rect = abs(Img_Ts_rect-comp_rect);
        img_frame_rect(i)=find(diff_rect== min(diff_rect));
    end

    %% %%%%%%%%%%%%%%%%%% Associate Image and BBox %%%%%%%%%%%%%%%% %%
    for i=1:length(image_data_rect)
        comp_rect = ones(size(Img_bbox_Ts_yolo))*(Img_Ts_rect(i));
        diff_rect = abs(Img_bbox_Ts_yolo-comp_rect);
        corr_yolo(i)=find(diff_rect== min(diff_rect),1,'first'); 
        error(i) = min(diff_rect);
    end
    %%
    [Xmin_bbox,Xmax_bbox,Ymin_bbox,Ymax_bbox,class_bb_bbox] = extract_YOLO(n_frames_mmWave,bbox_data_yolo,img_frame_rect,ids_p_frm);

    %% %%%%% Create Transformation Matrix %%%%%%%%%
    NewIM = [545.7881532041737, 0, 314.9817965777312; 0, 544.7004623674142,250.4216021567457; 0, 0, 1]';
    NewRD = [-0.4074079553767351, 0.1971963042143005];
    NewTD = [0.001847143363322204, -0.0009679199312377348];

    IntrinsicParams = cameraParameters('IntrinsicMatrix',NewIM,'RadialDistortion',NewRD,'TangentialDistortion',NewTD,'EstimateTangentialDistortion',logical(true)); 

    R1 = eye(3);
    t0=[0,-0.04,0]*1000;
    TM = cameraMatrix(IntrinsicParams,R1,t0);
    %% 
    M_track_r=[];
    epsilon=1.5;
    MinPts=8;
    track_id_r = 0;
    New_Track_r = zeros(n_frames_mmWave,256);
    Comb_Track_r = zeros(n_frames_mmWave,256);
    X_c_r = [];
    Y_c_r = [];
    Z_c_r = [];
    new_id_r=[];
    track_id_k = 0;
    M_track_k=[]
    X_c_k=[];
    Y_c_k=[];
    Xmax_bb_k=[];
    Xmin_bb_k=[];
    Ymax_bb_k=[];
    Ymin_bb_k=[];

    class_tracks = strings([1000,n_frames_mmWave]);
    X_ry_c=[];
    Y_ry_c=[];
    idx_ry=[];
    M_track_ry=[];
    track_id_ry=0;

    %%
    %h=figure('units','normalized','outerposition',[0 0 1 1]);
    for i=1:n_frames_mmWave

        [X_c_r,Y_c_r,Z_c_r,idx_img_r,M_track_r,track_id_r,New_Track_r,new_id_r,idx_r] = radar_DBSCAN_HA(TS1,i,TM,X_radar_mmWave(i,:),Y_radar_mmWave(i,:),Z_radar_mmWave(i,:),epsilon,MinPts,track_id_r,M_track_r,X_c_r,Y_c_r,Z_c_r,New_Track_r,new_id_r);

        rectified_image = readImage(image_data_rect{img_frame_rect(i)});

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
            X_min_crop(1:length(Track_id_yolo)) = Xmin_bbox(i,Track_id_yolo);
            Y_min_crop(1:length(Track_id_yolo)) = Ymin_bbox(i,Track_id_yolo);
            width_crop(1:length(Track_id_yolo)) = Xmax_bbox(i,Track_id_yolo)-Xmin_bbox(i,Track_id_yolo);
            height_crop(1:length(Track_id_yolo)) = Ymax_bbox(i,Track_id_yolo)-Ymin_bbox(i,Track_id_yolo);
        end

          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        C_ry = [];
        temp = [];
        temp_id = 0;
        C_new = [];
        idx_ry=[];
        tempidx_ry = [];
        buff_ry=[];

        if (~isempty(Track_id_rad) & ~isempty(Track_id_yolo))

                for k = 1:length(Track_id_rad)
                    C_ry(k,:) = vecnorm([X_proj_r(k)-X_proj_yolo; Y_proj_r(k)-Y_proj_yolo],2,1)';
                end
                [M_ry,u_new_ry,u_old_ry] = matchpairs(C_ry,150);

                for ii=1:length(M_ry(:,1))
                    X_radar_data=zeros(1,256);
                    Y_radar_data=zeros(1,256);
                    Z_radar_data=zeros(1,256);
                    Dop_radar_data =zeros(1,256);
                    SNR_radar_data =zeros(1,256);
                    Az_radar_data = zeros(1,256);
                    El_radar_data = zeros(1,256);
                    TS_radar_data = zeros(1,256);
                    track_id_ry = track_id_ry+1;
                    X_ry_c(track_id_ry) = (X_proj_r(M_ry(ii,1))+X_proj_yolo(M_ry(ii,2)))/2;
                    Y_ry_c(track_id_ry) = (Y_proj_r(M_ry(ii,1))+Y_proj_yolo(M_ry(ii,2)))/2;
                    class_tracks(Track_id_rad(M_ry(ii,1)),i)=Class_k_yolo(M_ry(ii,2));
                    M_track_ry = [M_track_ry;track_id_ry];
                    Comb_Track_r(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))))=track_id_ry;
                    X_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1)))))=X_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));
                    Y_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1)))))=Y_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));
                    Z_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1)))))=Z_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));
                    Dop_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))))) =Dop_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));
                    SNR_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))))) =SNR_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));
                    Az_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))))) = Az_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));
                    El_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))))) = El_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));
                    TS_radar_data(1,1:length(find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))))) = TS_radar_mmWave(i,find(New_Track_r(i,:)==Track_id_rad(M_ry(ii,1))));

                    J = [];
                    J = imcrop(rectified_image,[X_min_crop(M_ry(ii,2)),Y_min_crop(M_ry(ii,2)),width_crop(M_ry(ii,2)),height_crop(M_ry(ii,2))]);
                    J = imresize(J,[64 64]);

                    if (strcmp(Class_k_yolo(M_ry(ii,2)),'car')|strcmp(Class_k_yolo(M_ry(ii,2)),'truck')|strcmp(Class_k_yolo(M_ry(ii,2)),'bus'))
                        X_r_d=[X_r_d;X_radar_data];
                        Y_r_d=[Y_r_d;Y_radar_data];
                        Z_r_d=[Z_r_d;Z_radar_data];
                        Dop_r_d =[Dop_r_d;Dop_radar_data];
                        SNR_r_d =[SNR_r_d;SNR_radar_data];
                        Az_r_d = [Az_r_d;Az_radar_data];
                        El_r_d = [El_r_d;El_radar_data];
                        TS_r_d = [TS_r_d;TS_radar_data];
                        I_yolo_d(Idx,:,:,:) = J;
                        Label = [Label;1];
                        
                        
                        c_idx=c_idx+1;
                        Idx = Idx+1;
                    elseif strcmp(Class_k_yolo(M_ry(ii,2)),'person')
                        X_r_d=[X_r_d;X_radar_data];
                        Y_r_d=[Y_r_d;Y_radar_data];
                        Z_r_d=[Z_r_d;Z_radar_data];
                        Dop_r_d =[Dop_r_d;Dop_radar_data];
                        SNR_r_d =[SNR_r_d;SNR_radar_data];
                        Az_r_d = [Az_r_d;Az_radar_data];
                        El_r_d = [El_r_d;El_radar_data];
                        TS_r_d = [TS_r_d;TS_radar_data];
                        I_yolo_d(Idx,:,:,:) = J;
                        Label = [Label;2];
                        p_idx=p_idx+1;
                        Idx = Idx+1;
                    elseif strcmp(Class_k_yolo(M_ry(ii,2)),'bicycle')
                        X_r_d=[X_r_d;X_radar_data];
                        Y_r_d=[Y_r_d;Y_radar_data];
                        Z_r_d=[Z_r_d;Z_radar_data];
                        Dop_r_d =[Dop_r_d;Dop_radar_data];
                        SNR_r_d =[SNR_r_d;SNR_radar_data];
                        Az_r_d = [Az_r_d;Az_radar_data];
                        El_r_d = [El_r_d;El_radar_data];
                        TS_r_d = [TS_r_d;TS_radar_data];
                        I_yolo_d(Idx,:,:,:) = J;
                        Label = [Label;3];
                        b_idx=b_idx+1;
                        Idx = Idx+1;
                    elseif strcmp(Class_k_yolo(M_ry(ii,2)),'motorbike')
                        X_r_d=[X_r_d;X_radar_data];
                        Y_r_d=[Y_r_d;Y_radar_data];
                        Z_r_d=[Z_r_d;Z_radar_data];
                        Dop_r_d =[Dop_r_d;Dop_radar_data];
                        SNR_r_d =[SNR_r_d;SNR_radar_data];
                        Az_r_d = [Az_r_d;Az_radar_data];
                        El_r_d = [El_r_d;El_radar_data];
                        TS_r_d = [TS_r_d;TS_radar_data];
                        I_yolo_d(Idx,:,:,:) = J;
                        Label = [Label;4];
                        m_idx=m_idx+1;
                        Idx = Idx+1;
                    end
                     Frame_rad_1_r1 = [-Y_radar_data', -Z_radar_data', X_radar_data'];
                     worldPoints_rad_1_r1 = [Frame_rad_1_r1*1000 ones(size(Frame_rad_1_r1,1),1)];
                     projectedPoints_rad_1_r1 = worldPoints_rad_1_r1*TM;
                     X_proj_r1 = projectedPoints_rad_1_r1(:,1)./projectedPoints_rad_1_r1(:,3);
                     Y_proj_r1 = projectedPoints_rad_1_r1(:,2)./projectedPoints_rad_1_r1(:,3);
                     subplot(1,2,1)
                     imshow(rectified_image)
                     hold on
                     scatter(X_proj_r1,Y_proj_r1,10)
                     hold off
                     subplot(1,2,2)
                     imshow(J)
                     title(sprintf("Class:%s",Class_k_yolo(M_ry(ii,2))))
                     pause(0.01)
                end

        end


    end
end