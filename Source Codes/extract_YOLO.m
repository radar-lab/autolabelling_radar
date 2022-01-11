function [Xmin_bbox,Xmax_bbox,Ymin_bbox,Ymax_bbox,class_bb_bbox] = extract_YOLO(n_frames_mmWave,bbox_data_yolo,img_frame_rect,ids_p_frm)
    Xmin_bbox = zeros(n_frames_mmWave,10);
    Xmax_bbox = zeros(n_frames_mmWave,10);
    Ymin_bbox = zeros(n_frames_mmWave,10);
    Ymax_bbox = zeros(n_frames_mmWave,10);
    class_bb_bbox = strings([n_frames_mmWave,10]);

    for i=1:n_frames_mmWave

        idx_test = img_frame_rect(i);

        bbox_idx = nonzeros(ids_p_frm(idx_test,:));
        Xmin = [];
        Xmax = [];
        Ymin = [];
        Ymax = [];

        Xmin1 = [];
        Xmax1 = [];
        Ymin1 = [];
        Ymax1 = [];

        class_bb = string.empty;
        class_bb1= string.empty;
        bb_cnt = 1;
        for j_bbox=1:length(bbox_idx)
            num_classes = length(bbox_data_yolo{bbox_idx(j_bbox)}.BoundingBoxes_);
            for jj_box = 1:num_classes
                c_temp = bbox_data_yolo{bbox_idx(j_bbox)}.BoundingBoxes_(jj_box).Class1.Data;
            if (strcmp(c_temp,'car')|strcmp(c_temp,'truck')|strcmp(c_temp,'bus')|strcmp(c_temp,'person')|strcmp(c_temp,'bicycle')|strcmp(c_temp,'motorbike'))
                Xmin1(bb_cnt)= bbox_data_yolo{bbox_idx(j_bbox)}.BoundingBoxes_(jj_box).Xmin;
                Xmax1(bb_cnt)= bbox_data_yolo{bbox_idx(j_bbox)}.BoundingBoxes_(jj_box).Xmax;
                Ymin1(bb_cnt)= bbox_data_yolo{bbox_idx(j_bbox)}.BoundingBoxes_(jj_box).Ymin;
                Ymax1(bb_cnt)= bbox_data_yolo{bbox_idx(j_bbox)}.BoundingBoxes_(jj_box).Ymax;
                class_bb1(bb_cnt) = string(bbox_data_yolo{bbox_idx(j_bbox)}.BoundingBoxes_(jj_box).Class1.Data);
                bb_cnt = bb_cnt+1;
               %break
            end
            end
        end

        [~,i1,~] = unique(Xmin1);
        [~,i2,~] = unique(Xmax1);
        [~,i3,~] = unique(Ymin1);
        [~,i4,~] = unique(Ymax1);

        common_unique = intersect(intersect(i1,i2),intersect(i3,i4));

        Xmin(1:length(common_unique))= Xmin1(common_unique);
        Xmax(1:length(common_unique))= Xmax1(common_unique);
        Ymin(1:length(common_unique))= Ymin1(common_unique);
        Ymax(1:length(common_unique))= Ymax1(common_unique);
        class_bb(1:length(common_unique))= class_bb1(common_unique);

        bb_cnt(i) = length(common_unique)+1;

        Xmin_bbox(i,1:length(Xmin))= Xmin;
        Xmax_bbox(i,1:length(Xmax))= Xmax;
        Ymin_bbox(i,1:length(Ymin))= Ymin;
        Ymax_bbox(i,1:length(Ymax))= Ymax;
        class_bb_bbox(i,1:length(class_bb))= class_bb;


    end

end

