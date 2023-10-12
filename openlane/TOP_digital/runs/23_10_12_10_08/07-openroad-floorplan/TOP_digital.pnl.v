module TOP_digital (analog_trigger_in,
    analog_trigger_out,
    pwm_fall_out,
    pwm_rise_out,
    user_clock2,
    wb_clk_i,
    wb_rst_i,
    wbs_ack_o,
    wbs_cyc_i,
    wbs_stb_i,
    wbs_we_i,
    analog_io,
    io_in,
    la_data_in,
    la_oenb,
    user_irq,
    wbs_adr_i,
    wbs_dat_i,
    wbs_dat_o,
    wbs_sel_i);
 inout analog_trigger_in;
 inout analog_trigger_out;
 output pwm_fall_out;
 output pwm_rise_out;
 input user_clock2;
 input wb_clk_i;
 input wb_rst_i;
 output wbs_ack_o;
 input wbs_cyc_i;
 input wbs_stb_i;
 input wbs_we_i;
 inout [28:0] analog_io;
 input [37:0] io_in;
 input [127:0] la_data_in;
 input [127:0] la_oenb;
 output [2:0] user_irq;
 input [31:0] wbs_adr_i;
 input [31:0] wbs_dat_i;
 output [31:0] wbs_dat_o;
 input [3:0] wbs_sel_i;

 wire _000_;
 wire _001_;
 wire _002_;
 wire _003_;
 wire _004_;
 wire _005_;
 wire _006_;
 wire _007_;
 wire _008_;
 wire _009_;
 wire _010_;
 wire _011_;
 wire _012_;
 wire _013_;
 wire _014_;
 wire _015_;
 wire _016_;
 wire _017_;
 wire _018_;
 wire _019_;
 wire _020_;
 wire _021_;
 wire _022_;
 wire _023_;
 wire _024_;
 wire _025_;
 wire _026_;
 wire _027_;
 wire _028_;
 wire _029_;
 wire _030_;
 wire _031_;
 wire _032_;
 wire _033_;
 wire _034_;
 wire _035_;
 wire _036_;
 wire _037_;
 wire _038_;
 wire _039_;
 wire _040_;
 wire _041_;
 wire _042_;
 wire _043_;
 wire _044_;
 wire _045_;
 wire _046_;
 wire _047_;
 wire _048_;
 wire _049_;
 wire _050_;
 wire _051_;
 wire _052_;
 wire _053_;
 wire _054_;
 wire _055_;
 wire _056_;
 wire _057_;
 wire _058_;
 wire _059_;
 wire _060_;
 wire _061_;
 wire _062_;
 wire _063_;
 wire _064_;
 wire _065_;
 wire _066_;
 wire _067_;
 wire _068_;
 wire _069_;
 wire _070_;
 wire _071_;
 wire _072_;
 wire _073_;
 wire _074_;
 wire _075_;
 wire _076_;
 wire _077_;
 wire _078_;
 wire _079_;
 wire _080_;
 wire _081_;
 wire _082_;
 wire _083_;
 wire _084_;
 wire _085_;
 wire _086_;
 wire _087_;
 wire _088_;
 wire _089_;
 wire _090_;
 wire _091_;
 wire _092_;
 wire _093_;
 wire _094_;
 wire _095_;
 wire one_;
 wire \pwm_counter0.count[0] ;
 wire \pwm_counter0.count[1] ;
 wire \pwm_counter0.count[2] ;
 wire \pwm_counter0.count[3] ;
 wire \pwm_counter0.count[4] ;
 wire \pwm_counter0.count[5] ;
 wire \pwm_counter0.count[6] ;
 wire \pwm_counter0.count[7] ;
 wire \pwm_fall_detector.pos[0] ;
 wire \pwm_fall_detector.pos[1] ;
 wire \pwm_fall_detector.pos[2] ;
 wire \pwm_fall_detector.pos[3] ;
 wire \pwm_fall_detector.pos[4] ;
 wire \pwm_fall_detector.pos[5] ;
 wire \pwm_fall_detector.pos[6] ;
 wire \pwm_fall_detector.pos[7] ;
 wire \pwm_rise_detector.pos[0] ;
 wire \pwm_rise_detector.pos[1] ;
 wire \pwm_rise_detector.pos[2] ;
 wire \pwm_rise_detector.pos[3] ;
 wire \pwm_rise_detector.pos[4] ;
 wire \pwm_rise_detector.pos[5] ;
 wire \pwm_rise_detector.pos[6] ;
 wire \pwm_rise_detector.pos[7] ;
 wire zero_;

 sky130_fd_sc_hd__inv_2 _096_ (.A(\pwm_fall_detector.pos[4] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_092_));
 sky130_fd_sc_hd__inv_2 _097_ (.A(\pwm_rise_detector.pos[4] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_093_));
 sky130_fd_sc_hd__inv_2 _098_ (.A(\pwm_counter0.count[1] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_094_));
 sky130_fd_sc_hd__inv_2 _099_ (.A(\pwm_counter0.count[2] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_095_));
 sky130_fd_sc_hd__inv_2 _100_ (.A(\pwm_counter0.count[3] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_025_));
 sky130_fd_sc_hd__inv_2 _101_ (.A(\pwm_counter0.count[4] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_026_));
 sky130_fd_sc_hd__inv_2 _102_ (.A(\pwm_counter0.count[7] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_027_));
 sky130_fd_sc_hd__inv_2 _103_ (.A(la_data_in[0]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_028_));
 sky130_fd_sc_hd__a22o_2 _104_ (.A1(\pwm_rise_detector.pos[4] ),
    .A2(_026_),
    .B1(_027_),
    .B2(\pwm_rise_detector.pos[7] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_029_));
 sky130_fd_sc_hd__xor2_2 _105_ (.A(\pwm_rise_detector.pos[5] ),
    .B(\pwm_counter0.count[5] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_030_));
 sky130_fd_sc_hd__xor2_2 _106_ (.A(\pwm_rise_detector.pos[0] ),
    .B(\pwm_counter0.count[0] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_031_));
 sky130_fd_sc_hd__o22ai_2 _107_ (.A1(\pwm_rise_detector.pos[2] ),
    .A2(_095_),
    .B1(_027_),
    .B2(\pwm_rise_detector.pos[7] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_032_));
 sky130_fd_sc_hd__xnor2_2 _108_ (.A(\pwm_rise_detector.pos[6] ),
    .B(\pwm_counter0.count[6] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_033_));
 sky130_fd_sc_hd__a221o_2 _109_ (.A1(\pwm_rise_detector.pos[2] ),
    .A2(_095_),
    .B1(\pwm_counter0.count[4] ),
    .B2(_093_),
    .C1(_032_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_034_));
 sky130_fd_sc_hd__o221ai_2 _110_ (.A1(\pwm_rise_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_rise_detector.pos[3] ),
    .C1(_033_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_035_));
 sky130_fd_sc_hd__a221o_2 _111_ (.A1(\pwm_rise_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_rise_detector.pos[3] ),
    .C1(_031_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_036_));
 sky130_fd_sc_hd__or4_2 _112_ (.A(_029_),
    .B(_030_),
    .C(_035_),
    .D(_036_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_037_));
 sky130_fd_sc_hd__nor2_2 _113_ (.A(_034_),
    .B(_037_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(pwm_rise_out));
 sky130_fd_sc_hd__a22o_2 _114_ (.A1(\pwm_fall_detector.pos[4] ),
    .A2(_026_),
    .B1(_027_),
    .B2(\pwm_fall_detector.pos[7] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_038_));
 sky130_fd_sc_hd__xor2_2 _115_ (.A(\pwm_fall_detector.pos[5] ),
    .B(\pwm_counter0.count[5] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_039_));
 sky130_fd_sc_hd__xor2_2 _116_ (.A(\pwm_fall_detector.pos[0] ),
    .B(\pwm_counter0.count[0] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_040_));
 sky130_fd_sc_hd__o22ai_2 _117_ (.A1(\pwm_fall_detector.pos[2] ),
    .A2(_095_),
    .B1(_027_),
    .B2(\pwm_fall_detector.pos[7] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_041_));
 sky130_fd_sc_hd__xnor2_2 _118_ (.A(\pwm_fall_detector.pos[6] ),
    .B(\pwm_counter0.count[6] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_042_));
 sky130_fd_sc_hd__a221o_2 _119_ (.A1(\pwm_fall_detector.pos[2] ),
    .A2(_095_),
    .B1(\pwm_counter0.count[4] ),
    .B2(_092_),
    .C1(_041_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_043_));
 sky130_fd_sc_hd__o221ai_2 _120_ (.A1(\pwm_fall_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_fall_detector.pos[3] ),
    .C1(_042_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_044_));
 sky130_fd_sc_hd__a221o_2 _121_ (.A1(\pwm_fall_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_fall_detector.pos[3] ),
    .C1(_040_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_045_));
 sky130_fd_sc_hd__or4_2 _122_ (.A(_038_),
    .B(_039_),
    .C(_044_),
    .D(_045_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_046_));
 sky130_fd_sc_hd__nor2_2 _123_ (.A(_043_),
    .B(_046_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(pwm_fall_out));
 sky130_fd_sc_hd__nor2_2 _124_ (.A(\pwm_counter0.count[0] ),
    .B(la_data_in[0]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_000_));
 sky130_fd_sc_hd__o21ai_2 _125_ (.A1(\pwm_counter0.count[0] ),
    .A2(\pwm_counter0.count[1] ),
    .B1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_047_));
 sky130_fd_sc_hd__a21oi_2 _126_ (.A1(\pwm_counter0.count[0] ),
    .A2(\pwm_counter0.count[1] ),
    .B1(_047_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_001_));
 sky130_fd_sc_hd__and3_2 _127_ (.A(\pwm_counter0.count[0] ),
    .B(\pwm_counter0.count[1] ),
    .C(\pwm_counter0.count[2] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_048_));
 sky130_fd_sc_hd__a21oi_2 _128_ (.A1(\pwm_counter0.count[0] ),
    .A2(\pwm_counter0.count[1] ),
    .B1(\pwm_counter0.count[2] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_049_));
 sky130_fd_sc_hd__nor3_2 _129_ (.A(la_data_in[0]),
    .B(_048_),
    .C(_049_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_002_));
 sky130_fd_sc_hd__and4_2 _130_ (.A(\pwm_counter0.count[0] ),
    .B(\pwm_counter0.count[1] ),
    .C(\pwm_counter0.count[2] ),
    .D(\pwm_counter0.count[3] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_050_));
 sky130_fd_sc_hd__nor2_2 _131_ (.A(la_data_in[0]),
    .B(_050_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_051_));
 sky130_fd_sc_hd__o21a_2 _132_ (.A1(\pwm_counter0.count[3] ),
    .A2(_048_),
    .B1(_051_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_003_));
 sky130_fd_sc_hd__a21oi_2 _133_ (.A1(\pwm_counter0.count[4] ),
    .A2(_050_),
    .B1(la_data_in[0]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_052_));
 sky130_fd_sc_hd__o21a_2 _134_ (.A1(\pwm_counter0.count[4] ),
    .A2(_050_),
    .B1(_052_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_004_));
 sky130_fd_sc_hd__a21oi_2 _135_ (.A1(\pwm_counter0.count[4] ),
    .A2(_050_),
    .B1(\pwm_counter0.count[5] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_053_));
 sky130_fd_sc_hd__and3_2 _136_ (.A(\pwm_counter0.count[4] ),
    .B(\pwm_counter0.count[5] ),
    .C(_050_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_054_));
 sky130_fd_sc_hd__nor3_2 _137_ (.A(la_data_in[0]),
    .B(_053_),
    .C(_054_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_005_));
 sky130_fd_sc_hd__and4_2 _138_ (.A(\pwm_counter0.count[4] ),
    .B(\pwm_counter0.count[5] ),
    .C(\pwm_counter0.count[6] ),
    .D(_050_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_055_));
 sky130_fd_sc_hd__nor2_2 _139_ (.A(la_data_in[0]),
    .B(_055_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_056_));
 sky130_fd_sc_hd__o21a_2 _140_ (.A1(\pwm_counter0.count[6] ),
    .A2(_054_),
    .B1(_056_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_006_));
 sky130_fd_sc_hd__a21oi_2 _141_ (.A1(\pwm_counter0.count[7] ),
    .A2(_055_),
    .B1(la_data_in[0]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_057_));
 sky130_fd_sc_hd__o21a_2 _142_ (.A1(\pwm_counter0.count[7] ),
    .A2(_055_),
    .B1(_057_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_007_));
 sky130_fd_sc_hd__or4_2 _143_ (.A(wbs_adr_i[13]),
    .B(wbs_adr_i[12]),
    .C(wbs_adr_i[15]),
    .D(wbs_adr_i[14]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_058_));
 sky130_fd_sc_hd__or4_2 _144_ (.A(wbs_adr_i[9]),
    .B(wbs_adr_i[8]),
    .C(wbs_adr_i[11]),
    .D(wbs_adr_i[10]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_059_));
 sky130_fd_sc_hd__or4_2 _145_ (.A(wbs_adr_i[5]),
    .B(wbs_adr_i[4]),
    .C(wbs_adr_i[7]),
    .D(wbs_adr_i[6]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_060_));
 sky130_fd_sc_hd__or3_2 _146_ (.A(_058_),
    .B(_059_),
    .C(_060_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_061_));
 sky130_fd_sc_hd__or4_2 _147_ (.A(wbs_adr_i[21]),
    .B(wbs_adr_i[20]),
    .C(wbs_adr_i[23]),
    .D(wbs_adr_i[22]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_062_));
 sky130_fd_sc_hd__or4_2 _148_ (.A(wbs_adr_i[17]),
    .B(wbs_adr_i[16]),
    .C(wbs_adr_i[19]),
    .D(wbs_adr_i[18]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_063_));
 sky130_fd_sc_hd__or4bb_2 _149_ (.A(wbs_adr_i[31]),
    .B(wbs_adr_i[30]),
    .C_N(wbs_adr_i[29]),
    .D_N(wbs_adr_i[28]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_064_));
 sky130_fd_sc_hd__or4_2 _150_ (.A(wbs_adr_i[25]),
    .B(wbs_adr_i[24]),
    .C(wbs_adr_i[27]),
    .D(wbs_adr_i[26]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_065_));
 sky130_fd_sc_hd__nor4_2 _151_ (.A(_062_),
    .B(_063_),
    .C(_064_),
    .D(_065_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_066_));
 sky130_fd_sc_hd__nor2_2 _152_ (.A(wbs_adr_i[1]),
    .B(wbs_adr_i[0]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_067_));
 sky130_fd_sc_hd__or4_2 _153_ (.A(wbs_adr_i[1]),
    .B(wbs_adr_i[0]),
    .C(wbs_adr_i[3]),
    .D(wbs_adr_i[2]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_068_));
 sky130_fd_sc_hd__or4b_2 _154_ (.A(wbs_adr_i[1]),
    .B(wbs_adr_i[0]),
    .C(wbs_adr_i[3]),
    .D_N(wbs_adr_i[2]),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_069_));
 sky130_fd_sc_hd__and4bb_2 _155_ (.A_N(wbs_adr_i[3]),
    .B_N(_061_),
    .C(_066_),
    .D(_067_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_070_));
 sky130_fd_sc_hd__and3_2 _156_ (.A(wbs_stb_i),
    .B(_028_),
    .C(_070_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_008_));
 sky130_fd_sc_hd__and3_2 _157_ (.A(wbs_cyc_i),
    .B(wbs_stb_i),
    .C(wbs_we_i),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_071_));
 sky130_fd_sc_hd__nor4_2 _158_ (.A(_058_),
    .B(_059_),
    .C(_060_),
    .D(_068_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_072_));
 sky130_fd_sc_hd__nand3_2 _159_ (.A(_066_),
    .B(_071_),
    .C(_072_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_073_));
 sky130_fd_sc_hd__a31o_2 _160_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[0] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_074_));
 sky130_fd_sc_hd__o211a_2 _161_ (.A1(wbs_dat_i[0]),
    .A2(_073_),
    .B1(_074_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_009_));
 sky130_fd_sc_hd__a31o_2 _162_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[1] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_075_));
 sky130_fd_sc_hd__o211a_2 _163_ (.A1(wbs_dat_i[1]),
    .A2(_073_),
    .B1(_075_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_010_));
 sky130_fd_sc_hd__a31o_2 _164_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[2] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_076_));
 sky130_fd_sc_hd__o211a_2 _165_ (.A1(wbs_dat_i[2]),
    .A2(_073_),
    .B1(_076_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_011_));
 sky130_fd_sc_hd__a31o_2 _166_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[3] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_077_));
 sky130_fd_sc_hd__o211a_2 _167_ (.A1(wbs_dat_i[3]),
    .A2(_073_),
    .B1(_077_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_012_));
 sky130_fd_sc_hd__a31o_2 _168_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[4] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_078_));
 sky130_fd_sc_hd__o211a_2 _169_ (.A1(wbs_dat_i[4]),
    .A2(_073_),
    .B1(_078_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_013_));
 sky130_fd_sc_hd__a31o_2 _170_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[5] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_079_));
 sky130_fd_sc_hd__o211a_2 _171_ (.A1(wbs_dat_i[5]),
    .A2(_073_),
    .B1(_079_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_014_));
 sky130_fd_sc_hd__a31o_2 _172_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[6] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_080_));
 sky130_fd_sc_hd__o211a_2 _173_ (.A1(wbs_dat_i[6]),
    .A2(_073_),
    .B1(_080_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_015_));
 sky130_fd_sc_hd__a31o_2 _174_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[7] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_081_));
 sky130_fd_sc_hd__o211a_2 _175_ (.A1(wbs_dat_i[7]),
    .A2(_073_),
    .B1(_081_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_016_));
 sky130_fd_sc_hd__nor4_2 _176_ (.A(_058_),
    .B(_059_),
    .C(_060_),
    .D(_069_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_082_));
 sky130_fd_sc_hd__nand3_2 _177_ (.A(_066_),
    .B(_071_),
    .C(_082_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Y(_083_));
 sky130_fd_sc_hd__a31o_2 _178_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[0] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_084_));
 sky130_fd_sc_hd__o211a_2 _179_ (.A1(wbs_dat_i[0]),
    .A2(_083_),
    .B1(_084_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_017_));
 sky130_fd_sc_hd__a31o_2 _180_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[1] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_085_));
 sky130_fd_sc_hd__o211a_2 _181_ (.A1(wbs_dat_i[1]),
    .A2(_083_),
    .B1(_085_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_018_));
 sky130_fd_sc_hd__a31o_2 _182_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[2] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_086_));
 sky130_fd_sc_hd__o211a_2 _183_ (.A1(wbs_dat_i[2]),
    .A2(_083_),
    .B1(_086_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_019_));
 sky130_fd_sc_hd__a31o_2 _184_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[3] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_087_));
 sky130_fd_sc_hd__o211a_2 _185_ (.A1(wbs_dat_i[3]),
    .A2(_083_),
    .B1(_087_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_020_));
 sky130_fd_sc_hd__a31o_2 _186_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[4] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_088_));
 sky130_fd_sc_hd__o211a_2 _187_ (.A1(wbs_dat_i[4]),
    .A2(_083_),
    .B1(_088_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_021_));
 sky130_fd_sc_hd__a31o_2 _188_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[5] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_089_));
 sky130_fd_sc_hd__o211a_2 _189_ (.A1(wbs_dat_i[5]),
    .A2(_083_),
    .B1(_089_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_022_));
 sky130_fd_sc_hd__a31o_2 _190_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[6] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_090_));
 sky130_fd_sc_hd__o211a_2 _191_ (.A1(wbs_dat_i[6]),
    .A2(_083_),
    .B1(_090_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_023_));
 sky130_fd_sc_hd__a31o_2 _192_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[7] ),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_091_));
 sky130_fd_sc_hd__o211a_2 _193_ (.A1(wbs_dat_i[7]),
    .A2(_083_),
    .B1(_091_),
    .C1(_028_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(_024_));
 sky130_fd_sc_hd__dfxtp_2 _194_ (.CLK(wb_clk_i),
    .D(_000_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[0] ));
 sky130_fd_sc_hd__dfxtp_2 _195_ (.CLK(wb_clk_i),
    .D(_001_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[1] ));
 sky130_fd_sc_hd__dfxtp_2 _196_ (.CLK(wb_clk_i),
    .D(_002_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[2] ));
 sky130_fd_sc_hd__dfxtp_2 _197_ (.CLK(wb_clk_i),
    .D(_003_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[3] ));
 sky130_fd_sc_hd__dfxtp_2 _198_ (.CLK(wb_clk_i),
    .D(_004_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[4] ));
 sky130_fd_sc_hd__dfxtp_2 _199_ (.CLK(wb_clk_i),
    .D(_005_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[5] ));
 sky130_fd_sc_hd__dfxtp_2 _200_ (.CLK(wb_clk_i),
    .D(_006_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[6] ));
 sky130_fd_sc_hd__dfxtp_2 _201_ (.CLK(wb_clk_i),
    .D(_007_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_counter0.count[7] ));
 sky130_fd_sc_hd__dfxtp_2 _202_ (.CLK(wb_clk_i),
    .D(_008_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(wbs_ack_o));
 sky130_fd_sc_hd__dfxtp_2 _203_ (.CLK(wb_clk_i),
    .D(_009_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[0] ));
 sky130_fd_sc_hd__dfxtp_2 _204_ (.CLK(wb_clk_i),
    .D(_010_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[1] ));
 sky130_fd_sc_hd__dfxtp_2 _205_ (.CLK(wb_clk_i),
    .D(_011_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[2] ));
 sky130_fd_sc_hd__dfxtp_2 _206_ (.CLK(wb_clk_i),
    .D(_012_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[3] ));
 sky130_fd_sc_hd__dfxtp_2 _207_ (.CLK(wb_clk_i),
    .D(_013_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[4] ));
 sky130_fd_sc_hd__dfxtp_2 _208_ (.CLK(wb_clk_i),
    .D(_014_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[5] ));
 sky130_fd_sc_hd__dfxtp_2 _209_ (.CLK(wb_clk_i),
    .D(_015_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[6] ));
 sky130_fd_sc_hd__dfxtp_2 _210_ (.CLK(wb_clk_i),
    .D(_016_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_rise_detector.pos[7] ));
 sky130_fd_sc_hd__dfxtp_2 _211_ (.CLK(wb_clk_i),
    .D(_017_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[0] ));
 sky130_fd_sc_hd__dfxtp_2 _212_ (.CLK(wb_clk_i),
    .D(_018_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[1] ));
 sky130_fd_sc_hd__dfxtp_2 _213_ (.CLK(wb_clk_i),
    .D(_019_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[2] ));
 sky130_fd_sc_hd__dfxtp_2 _214_ (.CLK(wb_clk_i),
    .D(_020_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[3] ));
 sky130_fd_sc_hd__dfxtp_2 _215_ (.CLK(wb_clk_i),
    .D(_021_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[4] ));
 sky130_fd_sc_hd__dfxtp_2 _216_ (.CLK(wb_clk_i),
    .D(_022_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[5] ));
 sky130_fd_sc_hd__dfxtp_2 _217_ (.CLK(wb_clk_i),
    .D(_023_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[6] ));
 sky130_fd_sc_hd__dfxtp_2 _218_ (.CLK(wb_clk_i),
    .D(_024_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .Q(\pwm_fall_detector.pos[7] ));
 sky130_fd_sc_hd__buf_2 _219_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(user_irq[0]));
 sky130_fd_sc_hd__buf_2 _220_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(user_irq[1]));
 sky130_fd_sc_hd__buf_2 _221_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(user_irq[2]));
 sky130_fd_sc_hd__buf_2 _222_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[0]));
 sky130_fd_sc_hd__buf_2 _223_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[1]));
 sky130_fd_sc_hd__buf_2 _224_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[2]));
 sky130_fd_sc_hd__buf_2 _225_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[3]));
 sky130_fd_sc_hd__buf_2 _226_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[4]));
 sky130_fd_sc_hd__buf_2 _227_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[5]));
 sky130_fd_sc_hd__buf_2 _228_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[6]));
 sky130_fd_sc_hd__buf_2 _229_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[7]));
 sky130_fd_sc_hd__buf_2 _230_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[8]));
 sky130_fd_sc_hd__buf_2 _231_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[9]));
 sky130_fd_sc_hd__buf_2 _232_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[10]));
 sky130_fd_sc_hd__buf_2 _233_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[11]));
 sky130_fd_sc_hd__buf_2 _234_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[12]));
 sky130_fd_sc_hd__buf_2 _235_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[13]));
 sky130_fd_sc_hd__buf_2 _236_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[14]));
 sky130_fd_sc_hd__buf_2 _237_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[15]));
 sky130_fd_sc_hd__buf_2 _238_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[16]));
 sky130_fd_sc_hd__buf_2 _239_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[17]));
 sky130_fd_sc_hd__buf_2 _240_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[18]));
 sky130_fd_sc_hd__buf_2 _241_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[19]));
 sky130_fd_sc_hd__buf_2 _242_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[20]));
 sky130_fd_sc_hd__buf_2 _243_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[21]));
 sky130_fd_sc_hd__buf_2 _244_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[22]));
 sky130_fd_sc_hd__buf_2 _245_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[23]));
 sky130_fd_sc_hd__buf_2 _246_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[24]));
 sky130_fd_sc_hd__buf_2 _247_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[25]));
 sky130_fd_sc_hd__buf_2 _248_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[26]));
 sky130_fd_sc_hd__buf_2 _249_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[27]));
 sky130_fd_sc_hd__buf_2 _250_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[28]));
 sky130_fd_sc_hd__buf_2 _251_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[29]));
 sky130_fd_sc_hd__buf_2 _252_ (.A(zero_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[30]));
 sky130_fd_sc_hd__buf_2 _253_ (.A(one_),
    .VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .X(wbs_dat_o[31]));
 sky130_fd_sc_hd__conb_1 TIE_ZERO_zero_ (.VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .LO(zero_));
 sky130_fd_sc_hd__conb_1 TIE_ONE_one_ (.VGND(vssd1),
    .VNB(vssd1),
    .VPB(vccd1),
    .VPWR(vccd1),
    .HI(one_));
endmodule
