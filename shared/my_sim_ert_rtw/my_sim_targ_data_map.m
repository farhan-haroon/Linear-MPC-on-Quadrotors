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
        ;% Auto data (my_sim_P)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_P.Constant_Value
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_P.Constant_Value_h
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(2) = section;
            clear section

            section.nData     = 38;
            section.data(38)  = dumData; %prealloc

                    ;% my_sim_P.StateSpace_B_pr
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% my_sim_P.StateSpace_C_pr
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 4;

                    ;% my_sim_P.StateSpace_InitialCondition
                    section.data(3).logicalSrcIdx = 4;
                    section.data(3).dtTransOffset = 8;

                    ;% my_sim_P.SineWave_Amp
                    section.data(4).logicalSrcIdx = 5;
                    section.data(4).dtTransOffset = 12;

                    ;% my_sim_P.SineWave_Bias
                    section.data(5).logicalSrcIdx = 6;
                    section.data(5).dtTransOffset = 13;

                    ;% my_sim_P.SineWave_Freq
                    section.data(6).logicalSrcIdx = 7;
                    section.data(6).dtTransOffset = 14;

                    ;% my_sim_P.SineWave_Phase
                    section.data(7).logicalSrcIdx = 8;
                    section.data(7).dtTransOffset = 15;

                    ;% my_sim_P.Constant1_Value
                    section.data(8).logicalSrcIdx = 9;
                    section.data(8).dtTransOffset = 16;

                    ;% my_sim_P.Constant2_Value
                    section.data(9).logicalSrcIdx = 10;
                    section.data(9).dtTransOffset = 17;

                    ;% my_sim_P.Constant3_Value
                    section.data(10).logicalSrcIdx = 11;
                    section.data(10).dtTransOffset = 18;

                    ;% my_sim_P.last_x_InitialCondition
                    section.data(11).logicalSrcIdx = 12;
                    section.data(11).dtTransOffset = 19;

                    ;% my_sim_P.last_mv_InitialCondition
                    section.data(12).logicalSrcIdx = 13;
                    section.data(12).dtTransOffset = 23;

                    ;% my_sim_P.md_zero_Value
                    section.data(13).logicalSrcIdx = 14;
                    section.data(13).dtTransOffset = 27;

                    ;% my_sim_P.umin_zero_Value
                    section.data(14).logicalSrcIdx = 15;
                    section.data(14).dtTransOffset = 28;

                    ;% my_sim_P.umax_zero_Value
                    section.data(15).logicalSrcIdx = 16;
                    section.data(15).dtTransOffset = 32;

                    ;% my_sim_P.ymin_zero_Value
                    section.data(16).logicalSrcIdx = 17;
                    section.data(16).dtTransOffset = 36;

                    ;% my_sim_P.ymax_zero_Value
                    section.data(17).logicalSrcIdx = 18;
                    section.data(17).dtTransOffset = 40;

                    ;% my_sim_P.E_zero_Value
                    section.data(18).logicalSrcIdx = 19;
                    section.data(18).dtTransOffset = 44;

                    ;% my_sim_P.umin_scale4_Gain
                    section.data(19).logicalSrcIdx = 20;
                    section.data(19).dtTransOffset = 48;

                    ;% my_sim_P.F_zero_Value
                    section.data(20).logicalSrcIdx = 21;
                    section.data(20).dtTransOffset = 52;

                    ;% my_sim_P.ymin_scale1_Gain
                    section.data(21).logicalSrcIdx = 22;
                    section.data(21).dtTransOffset = 56;

                    ;% my_sim_P.G_zero_Value
                    section.data(22).logicalSrcIdx = 23;
                    section.data(22).dtTransOffset = 60;

                    ;% my_sim_P.S_zero_Value
                    section.data(23).logicalSrcIdx = 24;
                    section.data(23).dtTransOffset = 61;

                    ;% my_sim_P.ymin_scale2_Gain
                    section.data(24).logicalSrcIdx = 25;
                    section.data(24).dtTransOffset = 62;

                    ;% my_sim_P.switch_zero_Value
                    section.data(25).logicalSrcIdx = 26;
                    section.data(25).dtTransOffset = 63;

                    ;% my_sim_P.extmv_zero_Value
                    section.data(26).logicalSrcIdx = 27;
                    section.data(26).dtTransOffset = 64;

                    ;% my_sim_P.extmv_scale_Gain
                    section.data(27).logicalSrcIdx = 28;
                    section.data(27).dtTransOffset = 68;

                    ;% my_sim_P.mvtarget_zero_Value
                    section.data(28).logicalSrcIdx = 29;
                    section.data(28).dtTransOffset = 72;

                    ;% my_sim_P.extmv_scale1_Gain
                    section.data(29).logicalSrcIdx = 30;
                    section.data(29).dtTransOffset = 76;

                    ;% my_sim_P.ywt_zero_Value
                    section.data(30).logicalSrcIdx = 31;
                    section.data(30).dtTransOffset = 80;

                    ;% my_sim_P.uwt_zero_Value
                    section.data(31).logicalSrcIdx = 32;
                    section.data(31).dtTransOffset = 84;

                    ;% my_sim_P.duwt_zero_Value
                    section.data(32).logicalSrcIdx = 33;
                    section.data(32).dtTransOffset = 88;

                    ;% my_sim_P.ecrwt_zero_Value
                    section.data(33).logicalSrcIdx = 34;
                    section.data(33).dtTransOffset = 92;

                    ;% my_sim_P.umin_scale1_Gain
                    section.data(34).logicalSrcIdx = 35;
                    section.data(34).dtTransOffset = 93;

                    ;% my_sim_P.SimulationPace_P1
                    section.data(35).logicalSrcIdx = 36;
                    section.data(35).dtTransOffset = 97;

                    ;% my_sim_P.SimulationPace_P2
                    section.data(36).logicalSrcIdx = 37;
                    section.data(36).dtTransOffset = 98;

                    ;% my_sim_P.SimulationPace_P3
                    section.data(37).logicalSrcIdx = 38;
                    section.data(37).dtTransOffset = 99;

                    ;% my_sim_P.SimulationPace_P4
                    section.data(38).logicalSrcIdx = 39;
                    section.data(38).dtTransOffset = 100;

            nTotData = nTotData + section.nData;
            paramMap.sections(3) = section;
            clear section

            section.nData     = 4;
            section.data(4)  = dumData; %prealloc

                    ;% my_sim_P.StateSpace_B_ir
                    section.data(1).logicalSrcIdx = 40;
                    section.data(1).dtTransOffset = 0;

                    ;% my_sim_P.StateSpace_B_jc
                    section.data(2).logicalSrcIdx = 41;
                    section.data(2).dtTransOffset = 4;

                    ;% my_sim_P.StateSpace_C_ir
                    section.data(3).logicalSrcIdx = 42;
                    section.data(3).dtTransOffset = 9;

                    ;% my_sim_P.StateSpace_C_jc
                    section.data(4).logicalSrcIdx = 43;
                    section.data(4).dtTransOffset = 13;

            nTotData = nTotData + section.nData;
            paramMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_P.Constant4_Value
                    section.data(1).logicalSrcIdx = 44;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(5) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_P.Memory_InitialCondition
                    section.data(1).logicalSrcIdx = 45;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            paramMap.sections(6) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_P.Constant_Value_d
                    section.data(1).logicalSrcIdx = 46;
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
        nTotSects     = 4;
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
        ;% Auto data (my_sim_B)
        ;%
            section.nData     = 3;
            section.data(3)  = dumData; %prealloc

                    ;% my_sim_B.measuredoutput
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% my_sim_B.xk1
                    section.data(2).logicalSrcIdx = 6;
                    section.data(2).dtTransOffset = 4;

                    ;% my_sim_B.u
                    section.data(3).logicalSrcIdx = 7;
                    section.data(3).dtTransOffset = 8;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_B.iAout
                    section.data(1).logicalSrcIdx = 15;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(2) = section;
            clear section

            section.nData     = 5;
            section.data(5)  = dumData; %prealloc

                    ;% my_sim_B.x_ref
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

                    ;% my_sim_B.y_ref
                    section.data(2).logicalSrcIdx = 2;
                    section.data(2).dtTransOffset = 1;

                    ;% my_sim_B.z_ref
                    section.data(3).logicalSrcIdx = 3;
                    section.data(3).dtTransOffset = 2;

                    ;% my_sim_B.yaw_ref
                    section.data(4).logicalSrcIdx = 4;
                    section.data(4).dtTransOffset = 3;

                    ;% my_sim_B.umin_scale1
                    section.data(5).logicalSrcIdx = 5;
                    section.data(5).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            sigMap.sections(3) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_B.yaw_pos
                    section.data(1).logicalSrcIdx = 14;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            sigMap.sections(4) = section;
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
        nTotSects     = 5;
        sectIdxOffset = 4;

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
        ;% Auto data (my_sim_DW)
        ;%
            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% my_sim_DW.obj
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% my_sim_DW.obj_e
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% my_sim_DW.last_mv_DSTATE
                    section.data(1).logicalSrcIdx = 2;
                    section.data(1).dtTransOffset = 0;

                    ;% my_sim_DW.last_x_PreviousInput
                    section.data(2).logicalSrcIdx = 3;
                    section.data(2).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% my_sim_DW.Velocities_PWORK.LoggedData
                    section.data(1).logicalSrcIdx = 4;
                    section.data(1).dtTransOffset = 0;

                    ;% my_sim_DW.Positions_PWORK.LoggedData
                    section.data(2).logicalSrcIdx = 5;
                    section.data(2).dtTransOffset = 4;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_DW.is_active_c3_mpclib
                    section.data(1).logicalSrcIdx = 7;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section

            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% my_sim_DW.Memory_PreviousInput
                    section.data(1).logicalSrcIdx = 8;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(5) = section;
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


    targMap.checksum0 = 738916925;
    targMap.checksum1 = 3758009617;
    targMap.checksum2 = 1428962209;
    targMap.checksum3 = 4079769678;

