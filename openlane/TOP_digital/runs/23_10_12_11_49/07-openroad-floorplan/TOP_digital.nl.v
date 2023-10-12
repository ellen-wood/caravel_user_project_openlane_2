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
    io_oeb,
    io_out,
    la_data_in,
    la_data_out,
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
 output [37:0] io_oeb;
 output [37:0] io_out;
 input [127:0] la_data_in;
 output [127:0] la_data_out;
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
    .Y(_092_));
 sky130_fd_sc_hd__inv_2 _097_ (.A(\pwm_rise_detector.pos[4] ),
    .Y(_093_));
 sky130_fd_sc_hd__inv_2 _098_ (.A(\pwm_counter0.count[1] ),
    .Y(_094_));
 sky130_fd_sc_hd__inv_2 _099_ (.A(\pwm_counter0.count[2] ),
    .Y(_095_));
 sky130_fd_sc_hd__inv_2 _100_ (.A(\pwm_counter0.count[3] ),
    .Y(_025_));
 sky130_fd_sc_hd__inv_2 _101_ (.A(\pwm_counter0.count[4] ),
    .Y(_026_));
 sky130_fd_sc_hd__inv_2 _102_ (.A(\pwm_counter0.count[7] ),
    .Y(_027_));
 sky130_fd_sc_hd__inv_2 _103_ (.A(la_data_in[0]),
    .Y(_028_));
 sky130_fd_sc_hd__a22o_2 _104_ (.A1(\pwm_rise_detector.pos[4] ),
    .A2(_026_),
    .B1(_027_),
    .B2(\pwm_rise_detector.pos[7] ),
    .X(_029_));
 sky130_fd_sc_hd__xor2_2 _105_ (.A(\pwm_rise_detector.pos[5] ),
    .B(\pwm_counter0.count[5] ),
    .X(_030_));
 sky130_fd_sc_hd__xor2_2 _106_ (.A(\pwm_rise_detector.pos[0] ),
    .B(\pwm_counter0.count[0] ),
    .X(_031_));
 sky130_fd_sc_hd__o22ai_2 _107_ (.A1(\pwm_rise_detector.pos[2] ),
    .A2(_095_),
    .B1(_027_),
    .B2(\pwm_rise_detector.pos[7] ),
    .Y(_032_));
 sky130_fd_sc_hd__xnor2_2 _108_ (.A(\pwm_rise_detector.pos[6] ),
    .B(\pwm_counter0.count[6] ),
    .Y(_033_));
 sky130_fd_sc_hd__a221o_2 _109_ (.A1(\pwm_rise_detector.pos[2] ),
    .A2(_095_),
    .B1(\pwm_counter0.count[4] ),
    .B2(_093_),
    .C1(_032_),
    .X(_034_));
 sky130_fd_sc_hd__o221ai_2 _110_ (.A1(\pwm_rise_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_rise_detector.pos[3] ),
    .C1(_033_),
    .Y(_035_));
 sky130_fd_sc_hd__a221o_2 _111_ (.A1(\pwm_rise_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_rise_detector.pos[3] ),
    .C1(_031_),
    .X(_036_));
 sky130_fd_sc_hd__or4_2 _112_ (.A(_029_),
    .B(_030_),
    .C(_035_),
    .D(_036_),
    .X(_037_));
 sky130_fd_sc_hd__nor2_2 _113_ (.A(_034_),
    .B(_037_),
    .Y(pwm_rise_out));
 sky130_fd_sc_hd__a22o_2 _114_ (.A1(\pwm_fall_detector.pos[4] ),
    .A2(_026_),
    .B1(_027_),
    .B2(\pwm_fall_detector.pos[7] ),
    .X(_038_));
 sky130_fd_sc_hd__xor2_2 _115_ (.A(\pwm_fall_detector.pos[5] ),
    .B(\pwm_counter0.count[5] ),
    .X(_039_));
 sky130_fd_sc_hd__xor2_2 _116_ (.A(\pwm_fall_detector.pos[0] ),
    .B(\pwm_counter0.count[0] ),
    .X(_040_));
 sky130_fd_sc_hd__o22ai_2 _117_ (.A1(\pwm_fall_detector.pos[2] ),
    .A2(_095_),
    .B1(_027_),
    .B2(\pwm_fall_detector.pos[7] ),
    .Y(_041_));
 sky130_fd_sc_hd__xnor2_2 _118_ (.A(\pwm_fall_detector.pos[6] ),
    .B(\pwm_counter0.count[6] ),
    .Y(_042_));
 sky130_fd_sc_hd__a221o_2 _119_ (.A1(\pwm_fall_detector.pos[2] ),
    .A2(_095_),
    .B1(\pwm_counter0.count[4] ),
    .B2(_092_),
    .C1(_041_),
    .X(_043_));
 sky130_fd_sc_hd__o221ai_2 _120_ (.A1(\pwm_fall_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_fall_detector.pos[3] ),
    .C1(_042_),
    .Y(_044_));
 sky130_fd_sc_hd__a221o_2 _121_ (.A1(\pwm_fall_detector.pos[1] ),
    .A2(_094_),
    .B1(_025_),
    .B2(\pwm_fall_detector.pos[3] ),
    .C1(_040_),
    .X(_045_));
 sky130_fd_sc_hd__or4_2 _122_ (.A(_038_),
    .B(_039_),
    .C(_044_),
    .D(_045_),
    .X(_046_));
 sky130_fd_sc_hd__nor2_2 _123_ (.A(_043_),
    .B(_046_),
    .Y(pwm_fall_out));
 sky130_fd_sc_hd__nor2_2 _124_ (.A(\pwm_counter0.count[0] ),
    .B(la_data_in[0]),
    .Y(_000_));
 sky130_fd_sc_hd__o21ai_2 _125_ (.A1(\pwm_counter0.count[0] ),
    .A2(\pwm_counter0.count[1] ),
    .B1(_028_),
    .Y(_047_));
 sky130_fd_sc_hd__a21oi_2 _126_ (.A1(\pwm_counter0.count[0] ),
    .A2(\pwm_counter0.count[1] ),
    .B1(_047_),
    .Y(_001_));
 sky130_fd_sc_hd__and3_2 _127_ (.A(\pwm_counter0.count[0] ),
    .B(\pwm_counter0.count[1] ),
    .C(\pwm_counter0.count[2] ),
    .X(_048_));
 sky130_fd_sc_hd__a21oi_2 _128_ (.A1(\pwm_counter0.count[0] ),
    .A2(\pwm_counter0.count[1] ),
    .B1(\pwm_counter0.count[2] ),
    .Y(_049_));
 sky130_fd_sc_hd__nor3_2 _129_ (.A(la_data_in[0]),
    .B(_048_),
    .C(_049_),
    .Y(_002_));
 sky130_fd_sc_hd__and4_2 _130_ (.A(\pwm_counter0.count[0] ),
    .B(\pwm_counter0.count[1] ),
    .C(\pwm_counter0.count[2] ),
    .D(\pwm_counter0.count[3] ),
    .X(_050_));
 sky130_fd_sc_hd__nor2_2 _131_ (.A(la_data_in[0]),
    .B(_050_),
    .Y(_051_));
 sky130_fd_sc_hd__o21a_2 _132_ (.A1(\pwm_counter0.count[3] ),
    .A2(_048_),
    .B1(_051_),
    .X(_003_));
 sky130_fd_sc_hd__a21oi_2 _133_ (.A1(\pwm_counter0.count[4] ),
    .A2(_050_),
    .B1(la_data_in[0]),
    .Y(_052_));
 sky130_fd_sc_hd__o21a_2 _134_ (.A1(\pwm_counter0.count[4] ),
    .A2(_050_),
    .B1(_052_),
    .X(_004_));
 sky130_fd_sc_hd__a21oi_2 _135_ (.A1(\pwm_counter0.count[4] ),
    .A2(_050_),
    .B1(\pwm_counter0.count[5] ),
    .Y(_053_));
 sky130_fd_sc_hd__and3_2 _136_ (.A(\pwm_counter0.count[4] ),
    .B(\pwm_counter0.count[5] ),
    .C(_050_),
    .X(_054_));
 sky130_fd_sc_hd__nor3_2 _137_ (.A(la_data_in[0]),
    .B(_053_),
    .C(_054_),
    .Y(_005_));
 sky130_fd_sc_hd__and4_2 _138_ (.A(\pwm_counter0.count[4] ),
    .B(\pwm_counter0.count[5] ),
    .C(\pwm_counter0.count[6] ),
    .D(_050_),
    .X(_055_));
 sky130_fd_sc_hd__nor2_2 _139_ (.A(la_data_in[0]),
    .B(_055_),
    .Y(_056_));
 sky130_fd_sc_hd__o21a_2 _140_ (.A1(\pwm_counter0.count[6] ),
    .A2(_054_),
    .B1(_056_),
    .X(_006_));
 sky130_fd_sc_hd__a21oi_2 _141_ (.A1(\pwm_counter0.count[7] ),
    .A2(_055_),
    .B1(la_data_in[0]),
    .Y(_057_));
 sky130_fd_sc_hd__o21a_2 _142_ (.A1(\pwm_counter0.count[7] ),
    .A2(_055_),
    .B1(_057_),
    .X(_007_));
 sky130_fd_sc_hd__or4_2 _143_ (.A(wbs_adr_i[13]),
    .B(wbs_adr_i[12]),
    .C(wbs_adr_i[15]),
    .D(wbs_adr_i[14]),
    .X(_058_));
 sky130_fd_sc_hd__or4_2 _144_ (.A(wbs_adr_i[9]),
    .B(wbs_adr_i[8]),
    .C(wbs_adr_i[11]),
    .D(wbs_adr_i[10]),
    .X(_059_));
 sky130_fd_sc_hd__or4_2 _145_ (.A(wbs_adr_i[5]),
    .B(wbs_adr_i[4]),
    .C(wbs_adr_i[7]),
    .D(wbs_adr_i[6]),
    .X(_060_));
 sky130_fd_sc_hd__or3_2 _146_ (.A(_058_),
    .B(_059_),
    .C(_060_),
    .X(_061_));
 sky130_fd_sc_hd__or4_2 _147_ (.A(wbs_adr_i[21]),
    .B(wbs_adr_i[20]),
    .C(wbs_adr_i[23]),
    .D(wbs_adr_i[22]),
    .X(_062_));
 sky130_fd_sc_hd__or4_2 _148_ (.A(wbs_adr_i[17]),
    .B(wbs_adr_i[16]),
    .C(wbs_adr_i[19]),
    .D(wbs_adr_i[18]),
    .X(_063_));
 sky130_fd_sc_hd__or4bb_2 _149_ (.A(wbs_adr_i[31]),
    .B(wbs_adr_i[30]),
    .C_N(wbs_adr_i[29]),
    .D_N(wbs_adr_i[28]),
    .X(_064_));
 sky130_fd_sc_hd__or4_2 _150_ (.A(wbs_adr_i[25]),
    .B(wbs_adr_i[24]),
    .C(wbs_adr_i[27]),
    .D(wbs_adr_i[26]),
    .X(_065_));
 sky130_fd_sc_hd__nor4_2 _151_ (.A(_062_),
    .B(_063_),
    .C(_064_),
    .D(_065_),
    .Y(_066_));
 sky130_fd_sc_hd__nor2_2 _152_ (.A(wbs_adr_i[1]),
    .B(wbs_adr_i[0]),
    .Y(_067_));
 sky130_fd_sc_hd__or4_2 _153_ (.A(wbs_adr_i[1]),
    .B(wbs_adr_i[0]),
    .C(wbs_adr_i[3]),
    .D(wbs_adr_i[2]),
    .X(_068_));
 sky130_fd_sc_hd__or4b_2 _154_ (.A(wbs_adr_i[1]),
    .B(wbs_adr_i[0]),
    .C(wbs_adr_i[3]),
    .D_N(wbs_adr_i[2]),
    .X(_069_));
 sky130_fd_sc_hd__and4bb_2 _155_ (.A_N(wbs_adr_i[3]),
    .B_N(_061_),
    .C(_066_),
    .D(_067_),
    .X(_070_));
 sky130_fd_sc_hd__and3_2 _156_ (.A(wbs_stb_i),
    .B(_028_),
    .C(_070_),
    .X(_008_));
 sky130_fd_sc_hd__and3_2 _157_ (.A(wbs_cyc_i),
    .B(wbs_stb_i),
    .C(wbs_we_i),
    .X(_071_));
 sky130_fd_sc_hd__nor4_2 _158_ (.A(_058_),
    .B(_059_),
    .C(_060_),
    .D(_068_),
    .Y(_072_));
 sky130_fd_sc_hd__nand3_2 _159_ (.A(_066_),
    .B(_071_),
    .C(_072_),
    .Y(_073_));
 sky130_fd_sc_hd__a31o_2 _160_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[0] ),
    .X(_074_));
 sky130_fd_sc_hd__o211a_2 _161_ (.A1(wbs_dat_i[0]),
    .A2(_073_),
    .B1(_074_),
    .C1(_028_),
    .X(_009_));
 sky130_fd_sc_hd__a31o_2 _162_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[1] ),
    .X(_075_));
 sky130_fd_sc_hd__o211a_2 _163_ (.A1(wbs_dat_i[1]),
    .A2(_073_),
    .B1(_075_),
    .C1(_028_),
    .X(_010_));
 sky130_fd_sc_hd__a31o_2 _164_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[2] ),
    .X(_076_));
 sky130_fd_sc_hd__o211a_2 _165_ (.A1(wbs_dat_i[2]),
    .A2(_073_),
    .B1(_076_),
    .C1(_028_),
    .X(_011_));
 sky130_fd_sc_hd__a31o_2 _166_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[3] ),
    .X(_077_));
 sky130_fd_sc_hd__o211a_2 _167_ (.A1(wbs_dat_i[3]),
    .A2(_073_),
    .B1(_077_),
    .C1(_028_),
    .X(_012_));
 sky130_fd_sc_hd__a31o_2 _168_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[4] ),
    .X(_078_));
 sky130_fd_sc_hd__o211a_2 _169_ (.A1(wbs_dat_i[4]),
    .A2(_073_),
    .B1(_078_),
    .C1(_028_),
    .X(_013_));
 sky130_fd_sc_hd__a31o_2 _170_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[5] ),
    .X(_079_));
 sky130_fd_sc_hd__o211a_2 _171_ (.A1(wbs_dat_i[5]),
    .A2(_073_),
    .B1(_079_),
    .C1(_028_),
    .X(_014_));
 sky130_fd_sc_hd__a31o_2 _172_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[6] ),
    .X(_080_));
 sky130_fd_sc_hd__o211a_2 _173_ (.A1(wbs_dat_i[6]),
    .A2(_073_),
    .B1(_080_),
    .C1(_028_),
    .X(_015_));
 sky130_fd_sc_hd__a31o_2 _174_ (.A1(_066_),
    .A2(_071_),
    .A3(_072_),
    .B1(\pwm_rise_detector.pos[7] ),
    .X(_081_));
 sky130_fd_sc_hd__o211a_2 _175_ (.A1(wbs_dat_i[7]),
    .A2(_073_),
    .B1(_081_),
    .C1(_028_),
    .X(_016_));
 sky130_fd_sc_hd__nor4_2 _176_ (.A(_058_),
    .B(_059_),
    .C(_060_),
    .D(_069_),
    .Y(_082_));
 sky130_fd_sc_hd__nand3_2 _177_ (.A(_066_),
    .B(_071_),
    .C(_082_),
    .Y(_083_));
 sky130_fd_sc_hd__a31o_2 _178_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[0] ),
    .X(_084_));
 sky130_fd_sc_hd__o211a_2 _179_ (.A1(wbs_dat_i[0]),
    .A2(_083_),
    .B1(_084_),
    .C1(_028_),
    .X(_017_));
 sky130_fd_sc_hd__a31o_2 _180_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[1] ),
    .X(_085_));
 sky130_fd_sc_hd__o211a_2 _181_ (.A1(wbs_dat_i[1]),
    .A2(_083_),
    .B1(_085_),
    .C1(_028_),
    .X(_018_));
 sky130_fd_sc_hd__a31o_2 _182_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[2] ),
    .X(_086_));
 sky130_fd_sc_hd__o211a_2 _183_ (.A1(wbs_dat_i[2]),
    .A2(_083_),
    .B1(_086_),
    .C1(_028_),
    .X(_019_));
 sky130_fd_sc_hd__a31o_2 _184_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[3] ),
    .X(_087_));
 sky130_fd_sc_hd__o211a_2 _185_ (.A1(wbs_dat_i[3]),
    .A2(_083_),
    .B1(_087_),
    .C1(_028_),
    .X(_020_));
 sky130_fd_sc_hd__a31o_2 _186_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[4] ),
    .X(_088_));
 sky130_fd_sc_hd__o211a_2 _187_ (.A1(wbs_dat_i[4]),
    .A2(_083_),
    .B1(_088_),
    .C1(_028_),
    .X(_021_));
 sky130_fd_sc_hd__a31o_2 _188_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[5] ),
    .X(_089_));
 sky130_fd_sc_hd__o211a_2 _189_ (.A1(wbs_dat_i[5]),
    .A2(_083_),
    .B1(_089_),
    .C1(_028_),
    .X(_022_));
 sky130_fd_sc_hd__a31o_2 _190_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[6] ),
    .X(_090_));
 sky130_fd_sc_hd__o211a_2 _191_ (.A1(wbs_dat_i[6]),
    .A2(_083_),
    .B1(_090_),
    .C1(_028_),
    .X(_023_));
 sky130_fd_sc_hd__a31o_2 _192_ (.A1(_066_),
    .A2(_071_),
    .A3(_082_),
    .B1(\pwm_fall_detector.pos[7] ),
    .X(_091_));
 sky130_fd_sc_hd__o211a_2 _193_ (.A1(wbs_dat_i[7]),
    .A2(_083_),
    .B1(_091_),
    .C1(_028_),
    .X(_024_));
 sky130_fd_sc_hd__dfxtp_2 _194_ (.CLK(wb_clk_i),
    .D(_000_),
    .Q(\pwm_counter0.count[0] ));
 sky130_fd_sc_hd__dfxtp_2 _195_ (.CLK(wb_clk_i),
    .D(_001_),
    .Q(\pwm_counter0.count[1] ));
 sky130_fd_sc_hd__dfxtp_2 _196_ (.CLK(wb_clk_i),
    .D(_002_),
    .Q(\pwm_counter0.count[2] ));
 sky130_fd_sc_hd__dfxtp_2 _197_ (.CLK(wb_clk_i),
    .D(_003_),
    .Q(\pwm_counter0.count[3] ));
 sky130_fd_sc_hd__dfxtp_2 _198_ (.CLK(wb_clk_i),
    .D(_004_),
    .Q(\pwm_counter0.count[4] ));
 sky130_fd_sc_hd__dfxtp_2 _199_ (.CLK(wb_clk_i),
    .D(_005_),
    .Q(\pwm_counter0.count[5] ));
 sky130_fd_sc_hd__dfxtp_2 _200_ (.CLK(wb_clk_i),
    .D(_006_),
    .Q(\pwm_counter0.count[6] ));
 sky130_fd_sc_hd__dfxtp_2 _201_ (.CLK(wb_clk_i),
    .D(_007_),
    .Q(\pwm_counter0.count[7] ));
 sky130_fd_sc_hd__dfxtp_2 _202_ (.CLK(wb_clk_i),
    .D(_008_),
    .Q(wbs_ack_o));
 sky130_fd_sc_hd__dfxtp_2 _203_ (.CLK(wb_clk_i),
    .D(_009_),
    .Q(\pwm_rise_detector.pos[0] ));
 sky130_fd_sc_hd__dfxtp_2 _204_ (.CLK(wb_clk_i),
    .D(_010_),
    .Q(\pwm_rise_detector.pos[1] ));
 sky130_fd_sc_hd__dfxtp_2 _205_ (.CLK(wb_clk_i),
    .D(_011_),
    .Q(\pwm_rise_detector.pos[2] ));
 sky130_fd_sc_hd__dfxtp_2 _206_ (.CLK(wb_clk_i),
    .D(_012_),
    .Q(\pwm_rise_detector.pos[3] ));
 sky130_fd_sc_hd__dfxtp_2 _207_ (.CLK(wb_clk_i),
    .D(_013_),
    .Q(\pwm_rise_detector.pos[4] ));
 sky130_fd_sc_hd__dfxtp_2 _208_ (.CLK(wb_clk_i),
    .D(_014_),
    .Q(\pwm_rise_detector.pos[5] ));
 sky130_fd_sc_hd__dfxtp_2 _209_ (.CLK(wb_clk_i),
    .D(_015_),
    .Q(\pwm_rise_detector.pos[6] ));
 sky130_fd_sc_hd__dfxtp_2 _210_ (.CLK(wb_clk_i),
    .D(_016_),
    .Q(\pwm_rise_detector.pos[7] ));
 sky130_fd_sc_hd__dfxtp_2 _211_ (.CLK(wb_clk_i),
    .D(_017_),
    .Q(\pwm_fall_detector.pos[0] ));
 sky130_fd_sc_hd__dfxtp_2 _212_ (.CLK(wb_clk_i),
    .D(_018_),
    .Q(\pwm_fall_detector.pos[1] ));
 sky130_fd_sc_hd__dfxtp_2 _213_ (.CLK(wb_clk_i),
    .D(_019_),
    .Q(\pwm_fall_detector.pos[2] ));
 sky130_fd_sc_hd__dfxtp_2 _214_ (.CLK(wb_clk_i),
    .D(_020_),
    .Q(\pwm_fall_detector.pos[3] ));
 sky130_fd_sc_hd__dfxtp_2 _215_ (.CLK(wb_clk_i),
    .D(_021_),
    .Q(\pwm_fall_detector.pos[4] ));
 sky130_fd_sc_hd__dfxtp_2 _216_ (.CLK(wb_clk_i),
    .D(_022_),
    .Q(\pwm_fall_detector.pos[5] ));
 sky130_fd_sc_hd__dfxtp_2 _217_ (.CLK(wb_clk_i),
    .D(_023_),
    .Q(\pwm_fall_detector.pos[6] ));
 sky130_fd_sc_hd__dfxtp_2 _218_ (.CLK(wb_clk_i),
    .D(_024_),
    .Q(\pwm_fall_detector.pos[7] ));
 sky130_fd_sc_hd__buf_2 _219_ (.A(zero_),
    .X(la_data_out[68]));
 sky130_fd_sc_hd__buf_2 _220_ (.A(zero_),
    .X(la_data_out[69]));
 sky130_fd_sc_hd__buf_2 _221_ (.A(zero_),
    .X(la_data_out[70]));
 sky130_fd_sc_hd__buf_2 _222_ (.A(zero_),
    .X(la_data_out[71]));
 sky130_fd_sc_hd__buf_2 _223_ (.A(zero_),
    .X(la_data_out[72]));
 sky130_fd_sc_hd__buf_2 _224_ (.A(zero_),
    .X(la_data_out[73]));
 sky130_fd_sc_hd__buf_2 _225_ (.A(zero_),
    .X(la_data_out[74]));
 sky130_fd_sc_hd__buf_2 _226_ (.A(zero_),
    .X(la_data_out[75]));
 sky130_fd_sc_hd__buf_2 _227_ (.A(zero_),
    .X(la_data_out[76]));
 sky130_fd_sc_hd__buf_2 _228_ (.A(zero_),
    .X(la_data_out[77]));
 sky130_fd_sc_hd__buf_2 _229_ (.A(zero_),
    .X(la_data_out[78]));
 sky130_fd_sc_hd__buf_2 _230_ (.A(zero_),
    .X(la_data_out[79]));
 sky130_fd_sc_hd__buf_2 _231_ (.A(zero_),
    .X(la_data_out[80]));
 sky130_fd_sc_hd__buf_2 _232_ (.A(zero_),
    .X(la_data_out[81]));
 sky130_fd_sc_hd__buf_2 _233_ (.A(zero_),
    .X(la_data_out[82]));
 sky130_fd_sc_hd__buf_2 _234_ (.A(zero_),
    .X(la_data_out[83]));
 sky130_fd_sc_hd__buf_2 _235_ (.A(zero_),
    .X(la_data_out[84]));
 sky130_fd_sc_hd__buf_2 _236_ (.A(zero_),
    .X(la_data_out[85]));
 sky130_fd_sc_hd__buf_2 _237_ (.A(zero_),
    .X(la_data_out[86]));
 sky130_fd_sc_hd__buf_2 _238_ (.A(zero_),
    .X(la_data_out[87]));
 sky130_fd_sc_hd__buf_2 _239_ (.A(zero_),
    .X(la_data_out[88]));
 sky130_fd_sc_hd__buf_2 _240_ (.A(zero_),
    .X(la_data_out[89]));
 sky130_fd_sc_hd__buf_2 _241_ (.A(zero_),
    .X(la_data_out[90]));
 sky130_fd_sc_hd__buf_2 _242_ (.A(zero_),
    .X(la_data_out[91]));
 sky130_fd_sc_hd__buf_2 _243_ (.A(zero_),
    .X(la_data_out[92]));
 sky130_fd_sc_hd__buf_2 _244_ (.A(zero_),
    .X(la_data_out[93]));
 sky130_fd_sc_hd__buf_2 _245_ (.A(zero_),
    .X(la_data_out[94]));
 sky130_fd_sc_hd__buf_2 _246_ (.A(zero_),
    .X(la_data_out[95]));
 sky130_fd_sc_hd__buf_2 _247_ (.A(zero_),
    .X(la_data_out[96]));
 sky130_fd_sc_hd__buf_2 _248_ (.A(zero_),
    .X(la_data_out[97]));
 sky130_fd_sc_hd__buf_2 _249_ (.A(zero_),
    .X(la_data_out[98]));
 sky130_fd_sc_hd__buf_2 _250_ (.A(zero_),
    .X(la_data_out[99]));
 sky130_fd_sc_hd__buf_2 _251_ (.A(zero_),
    .X(la_data_out[100]));
 sky130_fd_sc_hd__buf_2 _252_ (.A(zero_),
    .X(la_data_out[101]));
 sky130_fd_sc_hd__buf_2 _253_ (.A(zero_),
    .X(la_data_out[102]));
 sky130_fd_sc_hd__buf_2 _254_ (.A(zero_),
    .X(la_data_out[103]));
 sky130_fd_sc_hd__buf_2 _255_ (.A(zero_),
    .X(la_data_out[104]));
 sky130_fd_sc_hd__buf_2 _256_ (.A(zero_),
    .X(la_data_out[105]));
 sky130_fd_sc_hd__buf_2 _257_ (.A(zero_),
    .X(la_data_out[106]));
 sky130_fd_sc_hd__buf_2 _258_ (.A(zero_),
    .X(la_data_out[107]));
 sky130_fd_sc_hd__buf_2 _259_ (.A(zero_),
    .X(la_data_out[108]));
 sky130_fd_sc_hd__buf_2 _260_ (.A(zero_),
    .X(la_data_out[109]));
 sky130_fd_sc_hd__buf_2 _261_ (.A(zero_),
    .X(la_data_out[110]));
 sky130_fd_sc_hd__buf_2 _262_ (.A(zero_),
    .X(la_data_out[111]));
 sky130_fd_sc_hd__buf_2 _263_ (.A(zero_),
    .X(la_data_out[112]));
 sky130_fd_sc_hd__buf_2 _264_ (.A(zero_),
    .X(la_data_out[113]));
 sky130_fd_sc_hd__buf_2 _265_ (.A(zero_),
    .X(la_data_out[114]));
 sky130_fd_sc_hd__buf_2 _266_ (.A(zero_),
    .X(la_data_out[115]));
 sky130_fd_sc_hd__buf_2 _267_ (.A(zero_),
    .X(la_data_out[116]));
 sky130_fd_sc_hd__buf_2 _268_ (.A(zero_),
    .X(la_data_out[117]));
 sky130_fd_sc_hd__buf_2 _269_ (.A(zero_),
    .X(la_data_out[118]));
 sky130_fd_sc_hd__buf_2 _270_ (.A(zero_),
    .X(la_data_out[119]));
 sky130_fd_sc_hd__buf_2 _271_ (.A(zero_),
    .X(la_data_out[120]));
 sky130_fd_sc_hd__buf_2 _272_ (.A(zero_),
    .X(la_data_out[121]));
 sky130_fd_sc_hd__buf_2 _273_ (.A(zero_),
    .X(la_data_out[122]));
 sky130_fd_sc_hd__buf_2 _274_ (.A(zero_),
    .X(la_data_out[123]));
 sky130_fd_sc_hd__buf_2 _275_ (.A(zero_),
    .X(la_data_out[124]));
 sky130_fd_sc_hd__buf_2 _276_ (.A(zero_),
    .X(la_data_out[125]));
 sky130_fd_sc_hd__buf_2 _277_ (.A(zero_),
    .X(la_data_out[126]));
 sky130_fd_sc_hd__buf_2 _278_ (.A(zero_),
    .X(la_data_out[127]));
 sky130_fd_sc_hd__buf_2 _279_ (.A(zero_),
    .X(user_irq[0]));
 sky130_fd_sc_hd__buf_2 _280_ (.A(zero_),
    .X(user_irq[1]));
 sky130_fd_sc_hd__buf_2 _281_ (.A(zero_),
    .X(user_irq[2]));
 sky130_fd_sc_hd__buf_2 _282_ (.A(one_),
    .X(wbs_dat_o[0]));
 sky130_fd_sc_hd__buf_2 _283_ (.A(zero_),
    .X(wbs_dat_o[1]));
 sky130_fd_sc_hd__buf_2 _284_ (.A(zero_),
    .X(wbs_dat_o[2]));
 sky130_fd_sc_hd__buf_2 _285_ (.A(zero_),
    .X(wbs_dat_o[3]));
 sky130_fd_sc_hd__buf_2 _286_ (.A(one_),
    .X(wbs_dat_o[4]));
 sky130_fd_sc_hd__buf_2 _287_ (.A(zero_),
    .X(wbs_dat_o[5]));
 sky130_fd_sc_hd__buf_2 _288_ (.A(zero_),
    .X(wbs_dat_o[6]));
 sky130_fd_sc_hd__buf_2 _289_ (.A(zero_),
    .X(wbs_dat_o[7]));
 sky130_fd_sc_hd__buf_2 _290_ (.A(one_),
    .X(wbs_dat_o[8]));
 sky130_fd_sc_hd__buf_2 _291_ (.A(one_),
    .X(wbs_dat_o[9]));
 sky130_fd_sc_hd__buf_2 _292_ (.A(zero_),
    .X(wbs_dat_o[10]));
 sky130_fd_sc_hd__buf_2 _293_ (.A(zero_),
    .X(wbs_dat_o[11]));
 sky130_fd_sc_hd__buf_2 _294_ (.A(one_),
    .X(wbs_dat_o[12]));
 sky130_fd_sc_hd__buf_2 _295_ (.A(one_),
    .X(wbs_dat_o[13]));
 sky130_fd_sc_hd__buf_2 _296_ (.A(zero_),
    .X(wbs_dat_o[14]));
 sky130_fd_sc_hd__buf_2 _297_ (.A(zero_),
    .X(wbs_dat_o[15]));
 sky130_fd_sc_hd__buf_2 _298_ (.A(one_),
    .X(wbs_dat_o[16]));
 sky130_fd_sc_hd__buf_2 _299_ (.A(zero_),
    .X(wbs_dat_o[17]));
 sky130_fd_sc_hd__buf_2 _300_ (.A(one_),
    .X(wbs_dat_o[18]));
 sky130_fd_sc_hd__buf_2 _301_ (.A(zero_),
    .X(wbs_dat_o[19]));
 sky130_fd_sc_hd__buf_2 _302_ (.A(one_),
    .X(wbs_dat_o[20]));
 sky130_fd_sc_hd__buf_2 _303_ (.A(zero_),
    .X(wbs_dat_o[21]));
 sky130_fd_sc_hd__buf_2 _304_ (.A(one_),
    .X(wbs_dat_o[22]));
 sky130_fd_sc_hd__buf_2 _305_ (.A(zero_),
    .X(wbs_dat_o[23]));
 sky130_fd_sc_hd__buf_2 _306_ (.A(zero_),
    .X(wbs_dat_o[24]));
 sky130_fd_sc_hd__buf_2 _307_ (.A(one_),
    .X(wbs_dat_o[25]));
 sky130_fd_sc_hd__buf_2 _308_ (.A(zero_),
    .X(wbs_dat_o[26]));
 sky130_fd_sc_hd__buf_2 _309_ (.A(one_),
    .X(wbs_dat_o[27]));
 sky130_fd_sc_hd__buf_2 _310_ (.A(zero_),
    .X(wbs_dat_o[28]));
 sky130_fd_sc_hd__buf_2 _311_ (.A(one_),
    .X(wbs_dat_o[29]));
 sky130_fd_sc_hd__buf_2 _312_ (.A(zero_),
    .X(wbs_dat_o[30]));
 sky130_fd_sc_hd__buf_2 _313_ (.A(one_),
    .X(wbs_dat_o[31]));
 sky130_fd_sc_hd__buf_2 _314_ (.A(zero_),
    .X(io_oeb[0]));
 sky130_fd_sc_hd__buf_2 _315_ (.A(zero_),
    .X(io_oeb[1]));
 sky130_fd_sc_hd__buf_2 _316_ (.A(zero_),
    .X(io_oeb[2]));
 sky130_fd_sc_hd__buf_2 _317_ (.A(zero_),
    .X(io_oeb[3]));
 sky130_fd_sc_hd__buf_2 _318_ (.A(zero_),
    .X(io_oeb[4]));
 sky130_fd_sc_hd__buf_2 _319_ (.A(zero_),
    .X(io_oeb[5]));
 sky130_fd_sc_hd__buf_2 _320_ (.A(zero_),
    .X(io_oeb[6]));
 sky130_fd_sc_hd__buf_2 _321_ (.A(zero_),
    .X(io_oeb[7]));
 sky130_fd_sc_hd__buf_2 _322_ (.A(zero_),
    .X(io_oeb[8]));
 sky130_fd_sc_hd__buf_2 _323_ (.A(zero_),
    .X(io_oeb[9]));
 sky130_fd_sc_hd__buf_2 _324_ (.A(zero_),
    .X(io_oeb[10]));
 sky130_fd_sc_hd__buf_2 _325_ (.A(zero_),
    .X(io_oeb[11]));
 sky130_fd_sc_hd__buf_2 _326_ (.A(zero_),
    .X(io_oeb[12]));
 sky130_fd_sc_hd__buf_2 _327_ (.A(zero_),
    .X(io_oeb[13]));
 sky130_fd_sc_hd__buf_2 _328_ (.A(zero_),
    .X(io_oeb[14]));
 sky130_fd_sc_hd__buf_2 _329_ (.A(zero_),
    .X(io_oeb[15]));
 sky130_fd_sc_hd__buf_2 _330_ (.A(zero_),
    .X(io_oeb[16]));
 sky130_fd_sc_hd__buf_2 _331_ (.A(zero_),
    .X(io_oeb[17]));
 sky130_fd_sc_hd__buf_2 _332_ (.A(zero_),
    .X(io_oeb[18]));
 sky130_fd_sc_hd__buf_2 _333_ (.A(zero_),
    .X(io_oeb[19]));
 sky130_fd_sc_hd__buf_2 _334_ (.A(zero_),
    .X(io_oeb[20]));
 sky130_fd_sc_hd__buf_2 _335_ (.A(zero_),
    .X(io_oeb[21]));
 sky130_fd_sc_hd__buf_2 _336_ (.A(zero_),
    .X(io_oeb[22]));
 sky130_fd_sc_hd__buf_2 _337_ (.A(zero_),
    .X(io_oeb[23]));
 sky130_fd_sc_hd__buf_2 _338_ (.A(zero_),
    .X(io_oeb[24]));
 sky130_fd_sc_hd__buf_2 _339_ (.A(zero_),
    .X(io_oeb[25]));
 sky130_fd_sc_hd__buf_2 _340_ (.A(zero_),
    .X(io_oeb[26]));
 sky130_fd_sc_hd__buf_2 _341_ (.A(zero_),
    .X(io_oeb[27]));
 sky130_fd_sc_hd__buf_2 _342_ (.A(zero_),
    .X(io_oeb[28]));
 sky130_fd_sc_hd__buf_2 _343_ (.A(zero_),
    .X(io_oeb[29]));
 sky130_fd_sc_hd__buf_2 _344_ (.A(zero_),
    .X(io_oeb[30]));
 sky130_fd_sc_hd__buf_2 _345_ (.A(zero_),
    .X(io_oeb[31]));
 sky130_fd_sc_hd__buf_2 _346_ (.A(zero_),
    .X(io_oeb[32]));
 sky130_fd_sc_hd__buf_2 _347_ (.A(zero_),
    .X(io_oeb[33]));
 sky130_fd_sc_hd__buf_2 _348_ (.A(zero_),
    .X(io_oeb[34]));
 sky130_fd_sc_hd__buf_2 _349_ (.A(zero_),
    .X(io_oeb[35]));
 sky130_fd_sc_hd__buf_2 _350_ (.A(zero_),
    .X(io_oeb[36]));
 sky130_fd_sc_hd__buf_2 _351_ (.A(zero_),
    .X(io_oeb[37]));
 sky130_fd_sc_hd__buf_2 _352_ (.A(zero_),
    .X(io_out[0]));
 sky130_fd_sc_hd__buf_2 _353_ (.A(zero_),
    .X(io_out[1]));
 sky130_fd_sc_hd__buf_2 _354_ (.A(zero_),
    .X(io_out[2]));
 sky130_fd_sc_hd__buf_2 _355_ (.A(zero_),
    .X(io_out[3]));
 sky130_fd_sc_hd__buf_2 _356_ (.A(zero_),
    .X(io_out[4]));
 sky130_fd_sc_hd__buf_2 _357_ (.A(zero_),
    .X(io_out[5]));
 sky130_fd_sc_hd__buf_2 _358_ (.A(zero_),
    .X(io_out[6]));
 sky130_fd_sc_hd__buf_2 _359_ (.A(zero_),
    .X(io_out[7]));
 sky130_fd_sc_hd__buf_2 _360_ (.A(zero_),
    .X(io_out[8]));
 sky130_fd_sc_hd__buf_2 _361_ (.A(zero_),
    .X(io_out[9]));
 sky130_fd_sc_hd__buf_2 _362_ (.A(zero_),
    .X(io_out[10]));
 sky130_fd_sc_hd__buf_2 _363_ (.A(zero_),
    .X(io_out[11]));
 sky130_fd_sc_hd__buf_2 _364_ (.A(zero_),
    .X(io_out[12]));
 sky130_fd_sc_hd__buf_2 _365_ (.A(zero_),
    .X(io_out[13]));
 sky130_fd_sc_hd__buf_2 _366_ (.A(zero_),
    .X(io_out[14]));
 sky130_fd_sc_hd__buf_2 _367_ (.A(zero_),
    .X(io_out[15]));
 sky130_fd_sc_hd__buf_2 _368_ (.A(zero_),
    .X(io_out[16]));
 sky130_fd_sc_hd__buf_2 _369_ (.A(zero_),
    .X(io_out[17]));
 sky130_fd_sc_hd__buf_2 _370_ (.A(zero_),
    .X(io_out[18]));
 sky130_fd_sc_hd__buf_2 _371_ (.A(zero_),
    .X(io_out[19]));
 sky130_fd_sc_hd__buf_2 _372_ (.A(zero_),
    .X(io_out[20]));
 sky130_fd_sc_hd__buf_2 _373_ (.A(zero_),
    .X(io_out[21]));
 sky130_fd_sc_hd__buf_2 _374_ (.A(zero_),
    .X(io_out[22]));
 sky130_fd_sc_hd__buf_2 _375_ (.A(zero_),
    .X(io_out[23]));
 sky130_fd_sc_hd__buf_2 _376_ (.A(zero_),
    .X(io_out[24]));
 sky130_fd_sc_hd__buf_2 _377_ (.A(zero_),
    .X(io_out[25]));
 sky130_fd_sc_hd__buf_2 _378_ (.A(zero_),
    .X(io_out[26]));
 sky130_fd_sc_hd__buf_2 _379_ (.A(zero_),
    .X(io_out[27]));
 sky130_fd_sc_hd__buf_2 _380_ (.A(zero_),
    .X(io_out[28]));
 sky130_fd_sc_hd__buf_2 _381_ (.A(zero_),
    .X(io_out[29]));
 sky130_fd_sc_hd__buf_2 _382_ (.A(zero_),
    .X(io_out[30]));
 sky130_fd_sc_hd__buf_2 _383_ (.A(zero_),
    .X(io_out[31]));
 sky130_fd_sc_hd__buf_2 _384_ (.A(zero_),
    .X(io_out[32]));
 sky130_fd_sc_hd__buf_2 _385_ (.A(zero_),
    .X(io_out[33]));
 sky130_fd_sc_hd__buf_2 _386_ (.A(zero_),
    .X(io_out[34]));
 sky130_fd_sc_hd__buf_2 _387_ (.A(zero_),
    .X(io_out[35]));
 sky130_fd_sc_hd__buf_2 _388_ (.A(zero_),
    .X(io_out[36]));
 sky130_fd_sc_hd__buf_2 _389_ (.A(zero_),
    .X(io_out[37]));
 sky130_fd_sc_hd__buf_2 _390_ (.A(zero_),
    .X(la_data_out[0]));
 sky130_fd_sc_hd__buf_2 _391_ (.A(zero_),
    .X(la_data_out[1]));
 sky130_fd_sc_hd__buf_2 _392_ (.A(zero_),
    .X(la_data_out[2]));
 sky130_fd_sc_hd__buf_2 _393_ (.A(zero_),
    .X(la_data_out[3]));
 sky130_fd_sc_hd__buf_2 _394_ (.A(zero_),
    .X(la_data_out[4]));
 sky130_fd_sc_hd__buf_2 _395_ (.A(zero_),
    .X(la_data_out[5]));
 sky130_fd_sc_hd__buf_2 _396_ (.A(zero_),
    .X(la_data_out[6]));
 sky130_fd_sc_hd__buf_2 _397_ (.A(zero_),
    .X(la_data_out[7]));
 sky130_fd_sc_hd__buf_2 _398_ (.A(zero_),
    .X(la_data_out[8]));
 sky130_fd_sc_hd__buf_2 _399_ (.A(zero_),
    .X(la_data_out[9]));
 sky130_fd_sc_hd__buf_2 _400_ (.A(zero_),
    .X(la_data_out[10]));
 sky130_fd_sc_hd__buf_2 _401_ (.A(zero_),
    .X(la_data_out[11]));
 sky130_fd_sc_hd__buf_2 _402_ (.A(zero_),
    .X(la_data_out[12]));
 sky130_fd_sc_hd__buf_2 _403_ (.A(zero_),
    .X(la_data_out[13]));
 sky130_fd_sc_hd__buf_2 _404_ (.A(zero_),
    .X(la_data_out[14]));
 sky130_fd_sc_hd__buf_2 _405_ (.A(zero_),
    .X(la_data_out[15]));
 sky130_fd_sc_hd__buf_2 _406_ (.A(zero_),
    .X(la_data_out[16]));
 sky130_fd_sc_hd__buf_2 _407_ (.A(zero_),
    .X(la_data_out[17]));
 sky130_fd_sc_hd__buf_2 _408_ (.A(zero_),
    .X(la_data_out[18]));
 sky130_fd_sc_hd__buf_2 _409_ (.A(zero_),
    .X(la_data_out[19]));
 sky130_fd_sc_hd__buf_2 _410_ (.A(zero_),
    .X(la_data_out[20]));
 sky130_fd_sc_hd__buf_2 _411_ (.A(zero_),
    .X(la_data_out[21]));
 sky130_fd_sc_hd__buf_2 _412_ (.A(zero_),
    .X(la_data_out[22]));
 sky130_fd_sc_hd__buf_2 _413_ (.A(zero_),
    .X(la_data_out[23]));
 sky130_fd_sc_hd__buf_2 _414_ (.A(zero_),
    .X(la_data_out[24]));
 sky130_fd_sc_hd__buf_2 _415_ (.A(zero_),
    .X(la_data_out[25]));
 sky130_fd_sc_hd__buf_2 _416_ (.A(zero_),
    .X(la_data_out[26]));
 sky130_fd_sc_hd__buf_2 _417_ (.A(zero_),
    .X(la_data_out[27]));
 sky130_fd_sc_hd__buf_2 _418_ (.A(zero_),
    .X(la_data_out[28]));
 sky130_fd_sc_hd__buf_2 _419_ (.A(zero_),
    .X(la_data_out[29]));
 sky130_fd_sc_hd__buf_2 _420_ (.A(zero_),
    .X(la_data_out[30]));
 sky130_fd_sc_hd__buf_2 _421_ (.A(zero_),
    .X(la_data_out[31]));
 sky130_fd_sc_hd__buf_2 _422_ (.A(zero_),
    .X(la_data_out[32]));
 sky130_fd_sc_hd__buf_2 _423_ (.A(zero_),
    .X(la_data_out[33]));
 sky130_fd_sc_hd__buf_2 _424_ (.A(zero_),
    .X(la_data_out[34]));
 sky130_fd_sc_hd__buf_2 _425_ (.A(zero_),
    .X(la_data_out[35]));
 sky130_fd_sc_hd__buf_2 _426_ (.A(zero_),
    .X(la_data_out[36]));
 sky130_fd_sc_hd__buf_2 _427_ (.A(zero_),
    .X(la_data_out[37]));
 sky130_fd_sc_hd__buf_2 _428_ (.A(zero_),
    .X(la_data_out[38]));
 sky130_fd_sc_hd__buf_2 _429_ (.A(zero_),
    .X(la_data_out[39]));
 sky130_fd_sc_hd__buf_2 _430_ (.A(zero_),
    .X(la_data_out[40]));
 sky130_fd_sc_hd__buf_2 _431_ (.A(zero_),
    .X(la_data_out[41]));
 sky130_fd_sc_hd__buf_2 _432_ (.A(zero_),
    .X(la_data_out[42]));
 sky130_fd_sc_hd__buf_2 _433_ (.A(zero_),
    .X(la_data_out[43]));
 sky130_fd_sc_hd__buf_2 _434_ (.A(zero_),
    .X(la_data_out[44]));
 sky130_fd_sc_hd__buf_2 _435_ (.A(zero_),
    .X(la_data_out[45]));
 sky130_fd_sc_hd__buf_2 _436_ (.A(zero_),
    .X(la_data_out[46]));
 sky130_fd_sc_hd__buf_2 _437_ (.A(zero_),
    .X(la_data_out[47]));
 sky130_fd_sc_hd__buf_2 _438_ (.A(zero_),
    .X(la_data_out[48]));
 sky130_fd_sc_hd__buf_2 _439_ (.A(zero_),
    .X(la_data_out[49]));
 sky130_fd_sc_hd__buf_2 _440_ (.A(zero_),
    .X(la_data_out[50]));
 sky130_fd_sc_hd__buf_2 _441_ (.A(zero_),
    .X(la_data_out[51]));
 sky130_fd_sc_hd__buf_2 _442_ (.A(zero_),
    .X(la_data_out[52]));
 sky130_fd_sc_hd__buf_2 _443_ (.A(zero_),
    .X(la_data_out[53]));
 sky130_fd_sc_hd__buf_2 _444_ (.A(zero_),
    .X(la_data_out[54]));
 sky130_fd_sc_hd__buf_2 _445_ (.A(zero_),
    .X(la_data_out[55]));
 sky130_fd_sc_hd__buf_2 _446_ (.A(zero_),
    .X(la_data_out[56]));
 sky130_fd_sc_hd__buf_2 _447_ (.A(zero_),
    .X(la_data_out[57]));
 sky130_fd_sc_hd__buf_2 _448_ (.A(zero_),
    .X(la_data_out[58]));
 sky130_fd_sc_hd__buf_2 _449_ (.A(zero_),
    .X(la_data_out[59]));
 sky130_fd_sc_hd__buf_2 _450_ (.A(zero_),
    .X(la_data_out[60]));
 sky130_fd_sc_hd__buf_2 _451_ (.A(zero_),
    .X(la_data_out[61]));
 sky130_fd_sc_hd__buf_2 _452_ (.A(zero_),
    .X(la_data_out[62]));
 sky130_fd_sc_hd__buf_2 _453_ (.A(zero_),
    .X(la_data_out[63]));
 sky130_fd_sc_hd__buf_2 _454_ (.A(zero_),
    .X(la_data_out[64]));
 sky130_fd_sc_hd__buf_2 _455_ (.A(zero_),
    .X(la_data_out[65]));
 sky130_fd_sc_hd__buf_2 _456_ (.A(zero_),
    .X(la_data_out[66]));
 sky130_fd_sc_hd__buf_2 _457_ (.A(zero_),
    .X(la_data_out[67]));
 sky130_fd_sc_hd__conb_1 TIE_ZERO_zero_ (.LO(zero_));
 sky130_fd_sc_hd__conb_1 TIE_ONE_one_ (.HI(one_));
endmodule
