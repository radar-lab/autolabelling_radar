function [TS1,n_frames_mmWave,X_radar_mmWave,Y_radar_mmWave,Z_radar_mmWave,SNR_radar_mmWave,TgtIdx_radar_mmWave,Dop_radar_mmWave,R_radar_mmWave,Vx_radar_mmWave,Vy_radar_mmWave,Vz_radar_mmWave,Az_radar_mmWave,El_radar_mmWave,TS_radar_mmWave] = extract_radar(radar_data)
    
    format long
    
        %% %%%%%%%%%%%%%% Extract mmWave Radar Messages %%%%%%%%%%%%% %%
    msg_id=1;
    for i=1:length(radar_data)

        PiD_mmWave(msg_id) = radar_data{i}.PointId;
        X_mmWave(msg_id) = radar_data{i}.X;
        Y_mmWave(msg_id) = radar_data{i}.Y;
        Z_mmWave(msg_id) = radar_data{i}.Z;
        SNR_mmWave(msg_id) = radar_data{i}.Snr;
        R_mmWave(msg_id) = radar_data{i}.Range;
        Dop_mmWave(msg_id) = radar_data{i}.Doppler;
        Vx_mmWave(msg_id) = radar_data{i}.VelX;
        Vy_mmWave(msg_id) = radar_data{i}.VelY;
        Vz_mmWave(msg_id) = radar_data{i}.VelZ;
        TS_mmWave(msg_id) = radar_data{i}.Header.Stamp.Sec+radar_data{i}.Header.Stamp.Nsec*1e-9;
        Az_mmWave(msg_id) = radar_data{i}.Angle;
        El_mmWave(msg_id) = radar_data{i}.Elev;
        X_c_mmWave(msg_id) = radar_data{i}.PosX;
        Y_c_mmWave(msg_id) = radar_data{i}.PosY;
        Z_c_mmWave(msg_id) = radar_data{i}.PosZ;
        TgtIdx_mmWave(msg_id) = radar_data{i}.TargetIdx;
        msg_id = msg_id+1;

    end
    %%
    % [A1,B1] = find(TgtIdx_mmWave>=253);
    % PiD_mmWave(B1) = [];
    % X_mmWave(B1) = [];
    % Y_mmWave(B1) = [];
    % Z_mmWave(B1) = [];
    % SNR_mmWave(B1) = [];
    % Dop_mmWave(B1) = [];
    % R_mmWave(B1) = [];
    % Vx_mmWave(B1) = [];
    % Vy_mmWave(B1) = [];
    % Vz_mmWave(B1) = [];
    % TS_mmWave(B1) = [];
    % Az_mmWave(B1) = [];
    % El_mmWave(B1) = [];
    % X_c_mmWave(B1) = [];
    % Y_c_mmWave(B1) = [];
    % Z_c_mmWave(B1) = [];
    % TgtIdx_mmWave(B1) = [];

    %% %%%%%%%%%%%%%% Divide the Radar Data into frames %%%%%%%%%%%% %% 
    % Find frame divisions based on the PointID
    % The idea is, if PointID(idx)>PointID(idx+1), then idx+1 is a _*marker*_ for 
    % a new frame

    [Values_mmWave, Markers_mmWave] = findpeaks(double(PiD_mmWave));
    Markers_mmWave = Markers_mmWave+1;

    n_frames_mmWave = length(Markers_mmWave);

    X_radar_mmWave = zeros(n_frames_mmWave,256);
    Y_radar_mmWave = zeros(n_frames_mmWave,256);
    Z_radar_mmWave = zeros(n_frames_mmWave,256);
    SNR_radar_mmWave = zeros(n_frames_mmWave,256);
    TgtIdx_radar_mmWave = zeros(n_frames_mmWave,256);
    Dop_radar_mmWave = zeros(n_frames_mmWave,256);
    R_radar_mmWave = zeros(n_frames_mmWave,256);
    Vx_radar_mmWave = zeros(n_frames_mmWave,256);
    Vy_radar_mmWave = zeros(n_frames_mmWave,256);
    Vz_radar_mmWave = zeros(n_frames_mmWave,256);
    Az_radar_mmWave = zeros(n_frames_mmWave,256);
    El_radar_mmWave = zeros(n_frames_mmWave,256);
    TS_radar_mmWave = zeros(n_frames_mmWave,256);

    for i=1:n_frames_mmWave
        TS1(i) = TS_mmWave(Markers_mmWave(i)-1);
        if i==1

          Y_radar_mmWave(i,1:Markers_mmWave(i)-1) = Y_mmWave(1:Markers_mmWave(i)-1);
          X_radar_mmWave(i,1:Markers_mmWave(i)-1) = X_mmWave(1:Markers_mmWave(i)-1);
          Z_radar_mmWave(i,1:Markers_mmWave(i)-1) = Z_mmWave(1:Markers_mmWave(i)-1);
          SNR_radar_mmWave(i,1:Markers_mmWave(i)-1) = SNR_mmWave(i,1:Markers_mmWave(i)-1);
          TgtIdx_radar_mmWave(i,1:Markers_mmWave(i)-1) = TgtIdx_mmWave(i,1:Markers_mmWave(i)-1);
          Dop_radar_mmWave(i,1:Markers_mmWave(i)-1) = Dop_mmWave(i,1:Markers_mmWave(i)-1);
          R_radar_mmWave(i,1:Markers_mmWave(i)-1) = R_mmWave(i,1:Markers_mmWave(i)-1);
          Vx_radar_mmWave(i,1:Markers_mmWave(i)-1) = Vx_mmWave(i,1:Markers_mmWave(i)-1);
          Vy_radar_mmWave(i,1:Markers_mmWave(i)-1) = Vy_mmWave(i,1:Markers_mmWave(i)-1);
          Vz_radar_mmWave(i,1:Markers_mmWave(i)-1) = Vz_mmWave(i,1:Markers_mmWave(i)-1);
          Az_radar_mmWave(i,1:Markers_mmWave(i)-1) = Az_mmWave(i,1:Markers_mmWave(i)-1);
          El_radar_mmWave(i,1:Markers_mmWave(i)-1) = El_mmWave(i,1:Markers_mmWave(i)-1);
          TS_radar_mmWave(i,1:Markers_mmWave(i)-1) = TS_mmWave(i,1:Markers_mmWave(i)-1);
        else

          Y_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = Y_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          X_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = X_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          Z_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = Z_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          SNR_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = SNR_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          TgtIdx_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = TgtIdx_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          Dop_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = Dop_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          R_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = R_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          Vx_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = Vx_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          Vy_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = Vy_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          Vz_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = Vz_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          Az_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = Az_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          El_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = El_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
          TS_radar_mmWave(i,1:length(Markers_mmWave(i-1):Markers_mmWave(i)-1)) = TS_mmWave(Markers_mmWave(i-1):Markers_mmWave(i)-1);
        end
    end

    
end

