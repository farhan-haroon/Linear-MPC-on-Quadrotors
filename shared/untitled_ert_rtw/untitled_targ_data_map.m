    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 7;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (untitled_P)
        ;%
            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% untitled_P.Ramp_InitialOutput
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_P.Ramp_slope
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% untitled_P.Ramp_start
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_P.Constant_Value
                    section.data(1).logicalSrcIdx = 3;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 41;
            section.data(41)  = dumData; %prealloc

                    ;% untitled_P.mv_vel_x_Y0
                    section.data(1).logicalSrcIdx = 4;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_P.mv_vel_y_Y0
                    section.data(2).logicalSrcIdx = 5;
                    section.data(2).dtTransOffset = 1;

                    ;% untitled_P.mv_vel_z_Y0
                    section.data(3).logicalSrcIdx = 6;
                    section.data(3).dtTransOffset = 2;

                    ;% untitled_P.mv_vel_yaw_Y0
                    section.data(4).logicalSrcIdx = 7;
                    section.data(4).dtTransOffset = 3;

                    ;% untitled_P.PlantStateSpace_B_pr
                    section.data(5).logicalSrcIdx = 8;
                    section.data(5).dtTransOffset = 4;

                    ;% untitled_P.PlantStateSpace_C_pr
                    section.data(6).logicalSrcIdx = 9;
                    section.data(6).dtTransOffset = 8;

                    ;% untitled_P.PlantStateSpace_InitialConditio
                    section.data(7).logicalSrcIdx = 10;
                    section.data(7).dtTransOffset = 12;

                    ;% untitled_P.Constant_Value_a
                    section.data(8).logicalSrcIdx = 11;
                    section.data(8).dtTransOffset = 16;

                    ;% untitled_P.Constant1_Value
                    section.data(9).logicalSrcIdx = 12;
                    section.data(9).dtTransOffset = 17;

                    ;% untitled_P.Constant2_Value
                    section.data(10).logicalSrcIdx = 13;
                    section.data(10).dtTransOffset = 18;

                    ;% untitled_P.G_zero_Value
                    section.data(11).logicalSrcIdx = 14;
                    section.data(11).dtTransOffset = 19;

                    ;% untitled_P.ywt_zero_Value
                    section.data(12).logicalSrcIdx = 15;
                    section.data(12).dtTransOffset = 20;

                    ;% untitled_P.uwt_zero_Value
                    section.data(13).logicalSrcIdx = 16;
                    section.data(13).dtTransOffset = 24;

                    ;% untitled_P.duwt_zero_Value
                    section.data(14).logicalSrcIdx = 17;
                    section.data(14).dtTransOffset = 28;

                    ;% untitled_P.extmv_zero_Value
                    section.data(15).logicalSrcIdx = 18;
                    section.data(15).dtTransOffset = 32;

                    ;% untitled_P.extmv_scale_Gain
                    section.data(16).logicalSrcIdx = 19;
                    section.data(16).dtTransOffset = 36;

                    ;% untitled_P.mvtarget_zero_Value
                    section.data(17).logicalSrcIdx = 20;
                    section.data(17).dtTransOffset = 40;

                    ;% untitled_P.extmv_scale1_Gain
                    section.data(18).logicalSrcIdx = 21;
                    section.data(18).dtTransOffset = 44;

                    ;% untitled_P.last_mv_InitialCondition
                    section.data(19).logicalSrcIdx = 22;
                    section.data(19).dtTransOffset = 48;

                    ;% untitled_P.last_x_InitialCondition
                    section.data(20).logicalSrcIdx = 23;
                    section.data(20).dtTransOffset = 52;

                    ;% untitled_P.Step_Y0
                    section.data(21).logicalSrcIdx = 24;
                    section.data(21).dtTransOffset = 56;

                    ;% untitled_P.md_zero_Value
                    section.data(22).logicalSrcIdx = 25;
                    section.data(22).dtTransOffset = 57;

                    ;% untitled_P.umin_zero_Value
                    section.data(23).logicalSrcIdx = 26;
                    section.data(23).dtTransOffset = 58;

                    ;% untitled_P.umax_zero_Value
                    section.data(24).logicalSrcIdx = 27;
                    section.data(24).dtTransOffset = 62;

                    ;% untitled_P.ymin_zero_Value
                    section.data(25).logicalSrcIdx = 28;
                    section.data(25).dtTransOffset = 66;

                    ;% untitled_P.ymax_zero_Value
                    section.data(26).logicalSrcIdx = 29;
                    section.data(26).dtTransOffset = 70;

                    ;% untitled_P.E_zero_Value
                    section.data(27).logicalSrcIdx = 30;
                    section.data(27).dtTransOffset = 74;

                    ;% untitled_P.umin_scale4_Gain
                    section.data(28).logicalSrcIdx = 31;
                    section.data(28).dtTransOffset = 78;

                    ;% untitled_P.F_zero_Value
                    section.data(29).logicalSrcIdx = 32;
                    section.data(29).dtTransOffset = 82;

                    ;% untitled_P.ymin_scale1_Gain
                    section.data(30).logicalSrcIdx = 33;
                    section.data(30).dtTransOffset = 86;

                    ;% untitled_P.S_zero_Value
                    section.data(31).logicalSrcIdx = 34;
                    section.data(31).dtTransOffset = 90;

                    ;% untitled_P.ymin_scale2_Gain
                    section.data(32).logicalSrcIdx = 35;
                    section.data(32).dtTransOffset = 91;

                    ;% untitled_P.switch_zero_Value
                    section.data(33).logicalSrcIdx = 36;
                    section.data(33).dtTransOffset = 92;

                    ;% untitled_P.ecrwt_zero_Value
                    section.data(34).logicalSrcIdx = 37;
                    section.data(34).dtTransOffset = 93;

                    ;% untitled_P.umin_scale1_Gain
                    section.data(35).logicalSrcIdx = 38;
                    section.data(35).dtTransOffset = 94;

                    ;% untitled_P.Constant_Value_m
                    section.data(36).logicalSrcIdx = 39;
                    section.data(36).dtTransOffset = 98;

                    ;% untitled_P.Constant1_Value_o
                    section.data(37).logicalSrcIdx = 40;
                    section.data(37).dtTransOffset = 99;

                    ;% untitled_P.SimulationPace_P1
                    section.data(38).logicalSrcIdx = 41;
                    section.data(38).dtTransOffset = 100;

                    ;% untitled_P.SimulationPace_P2
                    section.data(39).logicalSrcIdx = 42;
                    section.data(39).dtTransOffset = 101;

                    ;% untitled_P.SimulationPace_P3
                    section.data(40).logicalSrcIdx = 43;
                    section.data(40).dtTransOffset = 102;

                    ;% untitled_P.SimulationPace_P4
                    section.data(41).logicalSrcIdx = 44;
                    section.data(41).dtTransOffset = 103;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
            clear section

            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% untitled_P.PlantStateSpace_B_ir
                    section.data(1).logicalSrcIdx = 45;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_P.PlantStateSpace_B_jc
                    section.data(2).logicalSrcIdx = 46;
                    section.data(2).dtTransOffset = 4;

                    ;% untitled_P.PlantStateSpace_C_ir
                    section.data(3).logicalSrcIdx = 47;
                    section.data(3).dtTransOffset = 9;

                    ;% untitled_P.PlantStateSpace_C_jc
                    section.data(4).logicalSrcIdx = 48;
                    section.data(4).dtTransOffset = 13;

            nTotData = nTotData + section.nData;
            paramMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_P.TypeMask_Value
                    section.data(1).logicalSrcIdx = 49;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(5) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_P.Memory_InitialCondition
                    section.data(1).logicalSrcIdx = 50;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(6) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_P.CoordinateFrame_Value
                    section.data(1).logicalSrcIdx = 51;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(7) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 3;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (untitled_B)
        ;%
            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% untitled_B.measuredoutputs
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_B.umin_scale1
                    section.data(2).logicalSrcIdx = 2;
                    section.data(2).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_B.iAout
                    section.data(1).logicalSrcIdx = 14;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% untitled_B.mv_vel_y
                    section.data(1).logicalSrcIdx = 3;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_B.mv_vel_yaw
                    section.data(2).logicalSrcIdx = 4;
                    section.data(2).dtTransOffset = 1;

                    ;% untitled_B.mv_vel_z
                    section.data(3).logicalSrcIdx = 5;
                    section.data(3).dtTransOffset = 2;

                    ;% untitled_B.xk1
                    section.data(4).logicalSrcIdx = 6;
                    section.data(4).dtTransOffset = 3;

                    ;% untitled_B.u
                    section.data(5).logicalSrcIdx = 7;
                    section.data(5).dtTransOffset = 7;

            nTotData = nTotData + section.nData;
            sigMap.sections(3) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 6;
        sectIdxOffset = 3;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (untitled_DW)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_DW.obj
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% untitled_DW.last_mv_DSTATE
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_DW.last_x_PreviousInput
                    section.data(2).logicalSrcIdx = 2;
                    section.data(2).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_DW.Scope_PWORK.LoggedData
                    section.data(1).logicalSrcIdx = 3;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% untitled_DW.MPCController_SubsysRanBC
                    section.data(1).logicalSrcIdx = 5;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_DW.CommandVelocityPublisher_Subsys
                    section.data(2).logicalSrcIdx = 6;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% untitled_DW.is_active_c3_mpclib
                    section.data(1).logicalSrcIdx = 7;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% untitled_DW.Memory_PreviousInput
                    section.data(1).logicalSrcIdx = 8;
                    section.data(1).dtTransOffset = 0;

                    ;% untitled_DW.MPCController_MODE
                    section.data(2).logicalSrcIdx = 11;
                    section.data(2).dtTransOffset = 48;

            nTotData = nTotData + section.nData;
            dworkMap.sections(6) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 1104759370;
    targMap.checksum1 = 939934604;
    targMap.checksum2 = 3211798797;
    targMap.checksum3 = 4167976390;

