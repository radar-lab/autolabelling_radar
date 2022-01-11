function [c_idx,p_idx,b_idx,m_idx,X_r_d,Y_r_d,Z_r_d,Dop_r_d,SNR_r_d,Az_r_d,El_r_d,TS_r_d,Label,Idx_d] = gen_rad_only_data(bagfile,c_idx,p_idx,m_idx,b_idx,X_r_d,Y_r_d,Z_r_d,Dop_r_d,SNR_r_d,Az_r_d,El_r_d,TS_r_d,Label,Idx_d);


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
    %h=figure('units','normalized','outerposition',[0 0 1 1]);

    for i=1:n_frames_mmWave-1
        [X_c_r,Y_c_r,Z_c_r,idx_img_r,M_track_r,track_id_r,New_Track_r,new_id_r,idx_r] = radar_DBSCAN_HA(TS1,i,TM,X_radar_mmWave(i,:),Y_radar_mmWave(i,:),Z_radar_mmWave(i,:),epsilon,MinPts,track_id_r,M_track_r,X_c_r,Y_c_r,Z_c_r,New_Track_r,new_id_r);    

        [class_tracks] = only_class_new(i,class_bb_bbox,track_id_ry,New_Track_r,X_radar_mmWave,Y_radar_mmWave,Z_radar_mmWave,TM,Xmax_bbox,Ymax_bbox,Xmin_bbox,Ymin_bbox,class_tracks);
    end
    %%

    for i=1:max(max(New_Track_r))
        class_info=string;
        if sum(count(class_tracks(i,:),"bicycle"))>2
            class_info="bicycle";
            F_mode=2;
        elseif sum(count(class_tracks(i,:),"motorbike"))>3
            class_info="motorbike";
            F_mode=2;    
        elseif (sum(count(class_tracks(i,:),"person"))>3|((sum(count(class_tracks(i,:),"person"))/sum(count(class_tracks(i,:),"car")))>=0.3))
            class_info="person";
            F_mode=2;
        else
            [M_mode,F_mode]=mode(categorical(class_tracks(i,:)));
        
            if length(unique(class_tracks(i,:)))>1&&F_mode>=2
                class_info=string(M_mode);
            else
                class_info="No YOLO detection";
            end
        end
        [rows,columns]=find(New_Track_r==i);

        row_idx = sort(unique(rows));
        X_radar_data=zeros(length(unique(rows)),256);
        Y_radar_data=zeros(length(unique(rows)),256);
        Z_radar_data=zeros(length(unique(rows)),256);
        Dop_radar_data =zeros(length(unique(rows)),256);
        SNR_radar_data =zeros(length(unique(rows)),256);
        Az_radar_data = zeros(length(unique(rows)),256);
        El_radar_data = zeros(length(unique(rows)),256);
        TS_radar_data = zeros(length(unique(rows)),256);
        
        vid_k=1;
        if ~strcmp(class_info,'No YOLO detection')
        for j=1:length(unique(rows))
            if length(columns(rows==row_idx(j)))>=3
            X_radar_data(j,1:length(columns(rows==row_idx(j))))=X_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            Y_radar_data(j,1:length(columns(rows==row_idx(j))))=Y_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            Z_radar_data(j,1:length(columns(rows==row_idx(j))))=Z_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            Dop_radar_data(j,1:length(columns(rows==row_idx(j)))) =Dop_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            SNR_radar_data(j,1:length(columns(rows==row_idx(j)))) =SNR_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            Az_radar_data(j,1:length(columns(rows==row_idx(j)))) = Az_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            El_radar_data(j,1:length(columns(rows==row_idx(j)))) = El_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            TS_radar_data(j,1:length(columns(rows==row_idx(j)))) = TS_radar_mmWave(row_idx(j),columns(rows==row_idx(j)));
            end
        end
        
            
        if (strcmp(class_info,'car')|strcmp(class_info,'truck')|strcmp(class_info,'bus'))&&length(unique(rows))>=3
            for tt=1:length(unique(rows))-2
                X_r_d(Idx_d,:,:)=X_radar_data(tt:tt+2,:);
                Y_r_d(Idx_d,:,:)=Y_radar_data(tt:tt+2,:);
                Z_r_d(Idx_d,:,:)=Z_radar_data(tt:tt+2,:);
                Dop_r_d(Idx_d,:,:)=Dop_radar_data(tt:tt+2,:);
                SNR_r_d(Idx_d,:,:)=SNR_radar_data(tt:tt+2,:);
                Az_r_d(Idx_d,:,:)=Az_radar_data(tt:tt+2,:);
                El_r_d(Idx_d,:,:)=El_radar_data(tt:tt+2,:);
                TS_r_d(Idx_d,:,:)=TS_radar_data(tt:tt+2,:);
                Label(Idx_d)=1;
                Idx_d = Idx_d+1;
                c_idx=c_idx+1;
            end
                
        elseif strcmp(class_info,'person')&&length(unique(rows))>=5
            for tt=1:length(unique(rows))-2
                X_r_d(Idx_d,:,:)=X_radar_data(tt:tt+2,:);
                Y_r_d(Idx_d,:,:)=Y_radar_data(tt:tt+2,:);
                Z_r_d(Idx_d,:,:)=Z_radar_data(tt:tt+2,:);
                Dop_r_d(Idx_d,:,:)=Dop_radar_data(tt:tt+2,:);
                SNR_r_d(Idx_d,:,:)=SNR_radar_data(tt:tt+2,:);
                Az_r_d(Idx_d,:,:)=Az_radar_data(tt:tt+2,:);
                El_r_d(Idx_d,:,:)=El_radar_data(tt:tt+2,:);
                TS_r_d(Idx_d,:,:)=TS_radar_data(tt:tt+2,:);
                Label(Idx_d)=2;
                Idx_d = Idx_d+1;
                p_idx=p_idx+1;
            end
        elseif strcmp(class_info,'bicycle')&&length(unique(rows))>=3
            for tt=1:length(unique(rows))-2
                X_r_d(Idx_d,:,:)=X_radar_data(tt:tt+2,:);
                Y_r_d(Idx_d,:,:)=Y_radar_data(tt:tt+2,:);
                Z_r_d(Idx_d,:,:)=Z_radar_data(tt:tt+2,:);
                Dop_r_d(Idx_d,:,:)=Dop_radar_data(tt:tt+2,:);
                SNR_r_d(Idx_d,:,:)=SNR_radar_data(tt:tt+2,:);
                Az_r_d(Idx_d,:,:)=Az_radar_data(tt:tt+2,:);
                El_r_d(Idx_d,:,:)=El_radar_data(tt:tt+2,:);
                TS_r_d(Idx_d,:,:)=TS_radar_data(tt:tt+2,:);
                Label(Idx_d)=3;
                Idx_d = Idx_d+1;
                b_idx=b_idx+1;
            end
        elseif strcmp(class_info,'motorbike')&&length(unique(rows))>=3
            for tt=1:length(unique(rows))-2
                X_r_d(Idx_d,:,:)=X_radar_data(tt:tt+2,:);
                Y_r_d(Idx_d,:,:)=Y_radar_data(tt:tt+2,:);
                Z_r_d(Idx_d,:,:)=Z_radar_data(tt:tt+2,:);
                Dop_r_d(Idx_d,:,:)=Dop_radar_data(tt:tt+2,:);
                SNR_r_d(Idx_d,:,:)=SNR_radar_data(tt:tt+2,:);
                Az_r_d(Idx_d,:,:)=Az_radar_data(tt:tt+2,:);
                El_r_d(Idx_d,:,:)=El_radar_data(tt:tt+2,:);
                TS_r_d(Idx_d,:,:)=TS_radar_data(tt:tt+2,:);
                Label(Idx_d)=4;
                Idx_d = Idx_d+1;
                m_idx=m_idx+1;
            end

        end
     end
   end

end

