-- ==============================================================
-- Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
-- Tool Version Limit: 2024.11
-- Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
-- 
-- ==============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD.all;

entity s2m_vmap_control_s_axi is
generic (
    C_S_AXI_ADDR_WIDTH    : INTEGER := 8;
    C_S_AXI_DATA_WIDTH    : INTEGER := 32);
port (
    ACLK                  :in   STD_LOGIC;
    ARESET                :in   STD_LOGIC;
    ACLK_EN               :in   STD_LOGIC;
    AWADDR                :in   STD_LOGIC_VECTOR(C_S_AXI_ADDR_WIDTH-1 downto 0);
    AWVALID               :in   STD_LOGIC;
    AWREADY               :out  STD_LOGIC;
    WDATA                 :in   STD_LOGIC_VECTOR(C_S_AXI_DATA_WIDTH-1 downto 0);
    WSTRB                 :in   STD_LOGIC_VECTOR(C_S_AXI_DATA_WIDTH/8-1 downto 0);
    WVALID                :in   STD_LOGIC;
    WREADY                :out  STD_LOGIC;
    BRESP                 :out  STD_LOGIC_VECTOR(1 downto 0);
    BVALID                :out  STD_LOGIC;
    BREADY                :in   STD_LOGIC;
    ARADDR                :in   STD_LOGIC_VECTOR(C_S_AXI_ADDR_WIDTH-1 downto 0);
    ARVALID               :in   STD_LOGIC;
    ARREADY               :out  STD_LOGIC;
    RDATA                 :out  STD_LOGIC_VECTOR(C_S_AXI_DATA_WIDTH-1 downto 0);
    RRESP                 :out  STD_LOGIC_VECTOR(1 downto 0);
    RVALID                :out  STD_LOGIC;
    RREADY                :in   STD_LOGIC;
    interrupt             :out  STD_LOGIC;
    q                     :out  STD_LOGIC_VECTOR(63 downto 0);
    vbw                   :out  STD_LOGIC_VECTOR(63 downto 0);
    tab                   :out  STD_LOGIC_VECTOR(63 downto 0);
    parq                  :out  STD_LOGIC_VECTOR(63 downto 0);
    parm                  :out  STD_LOGIC_VECTOR(63 downto 0);
    Hb                    :out  STD_LOGIC_VECTOR(63 downto 0);
    nq                    :out  STD_LOGIC_VECTOR(31 downto 0);
    mode                  :out  STD_LOGIC_VECTOR(31 downto 0);
    L                     :out  STD_LOGIC_VECTOR(31 downto 0);
    invL                  :out  STD_LOGIC_VECTOR(31 downto 0);
    rmax                  :out  STD_LOGIC_VECTOR(31 downto 0);
    orgx                  :out  STD_LOGIC_VECTOR(31 downto 0);
    orgy                  :out  STD_LOGIC_VECTOR(31 downto 0);
    orgz                  :out  STD_LOGIC_VECTOR(31 downto 0);
    dimx                  :out  STD_LOGIC_VECTOR(31 downto 0);
    dimy                  :out  STD_LOGIC_VECTOR(31 downto 0);
    dimz                  :out  STD_LOGIC_VECTOR(31 downto 0);
    rescan                :out  STD_LOGIC_VECTOR(31 downto 0);
    ap_start              :out  STD_LOGIC;
    ap_done               :in   STD_LOGIC;
    ap_ready              :in   STD_LOGIC;
    ap_idle               :in   STD_LOGIC
);
end entity s2m_vmap_control_s_axi;

-- ------------------------Address Info-------------------
-- Protocol Used: ap_ctrl_hs
--
-- 0x00 : Control signals
--        bit 0  - ap_start (Read/Write/COH)
--        bit 1  - ap_done (Read/COR)
--        bit 2  - ap_idle (Read)
--        bit 3  - ap_ready (Read/COR)
--        bit 7  - auto_restart (Read/Write)
--        bit 9  - interrupt (Read)
--        others - reserved
-- 0x04 : Global Interrupt Enable Register
--        bit 0  - Global Interrupt Enable (Read/Write)
--        others - reserved
-- 0x08 : IP Interrupt Enable Register (Read/Write)
--        bit 0 - enable ap_done interrupt (Read/Write)
--        bit 1 - enable ap_ready interrupt (Read/Write)
--        others - reserved
-- 0x0c : IP Interrupt Status Register (Read/TOW)
--        bit 0 - ap_done (Read/TOW)
--        bit 1 - ap_ready (Read/TOW)
--        others - reserved
-- 0x10 : Data signal of q
--        bit 31~0 - q[31:0] (Read/Write)
-- 0x14 : Data signal of q
--        bit 31~0 - q[63:32] (Read/Write)
-- 0x18 : reserved
-- 0x1c : Data signal of vbw
--        bit 31~0 - vbw[31:0] (Read/Write)
-- 0x20 : Data signal of vbw
--        bit 31~0 - vbw[63:32] (Read/Write)
-- 0x24 : reserved
-- 0x28 : Data signal of tab
--        bit 31~0 - tab[31:0] (Read/Write)
-- 0x2c : Data signal of tab
--        bit 31~0 - tab[63:32] (Read/Write)
-- 0x30 : reserved
-- 0x34 : Data signal of parq
--        bit 31~0 - parq[31:0] (Read/Write)
-- 0x38 : Data signal of parq
--        bit 31~0 - parq[63:32] (Read/Write)
-- 0x3c : reserved
-- 0x40 : Data signal of parm
--        bit 31~0 - parm[31:0] (Read/Write)
-- 0x44 : Data signal of parm
--        bit 31~0 - parm[63:32] (Read/Write)
-- 0x48 : reserved
-- 0x4c : Data signal of Hb
--        bit 31~0 - Hb[31:0] (Read/Write)
-- 0x50 : Data signal of Hb
--        bit 31~0 - Hb[63:32] (Read/Write)
-- 0x54 : reserved
-- 0x58 : Data signal of nq
--        bit 31~0 - nq[31:0] (Read/Write)
-- 0x5c : reserved
-- 0x60 : Data signal of mode
--        bit 31~0 - mode[31:0] (Read/Write)
-- 0x64 : reserved
-- 0x68 : Data signal of L
--        bit 31~0 - L[31:0] (Read/Write)
-- 0x6c : reserved
-- 0x70 : Data signal of invL
--        bit 31~0 - invL[31:0] (Read/Write)
-- 0x74 : reserved
-- 0x78 : Data signal of rmax
--        bit 31~0 - rmax[31:0] (Read/Write)
-- 0x7c : reserved
-- 0x80 : Data signal of orgx
--        bit 31~0 - orgx[31:0] (Read/Write)
-- 0x84 : reserved
-- 0x88 : Data signal of orgy
--        bit 31~0 - orgy[31:0] (Read/Write)
-- 0x8c : reserved
-- 0x90 : Data signal of orgz
--        bit 31~0 - orgz[31:0] (Read/Write)
-- 0x94 : reserved
-- 0x98 : Data signal of dimx
--        bit 31~0 - dimx[31:0] (Read/Write)
-- 0x9c : reserved
-- 0xa0 : Data signal of dimy
--        bit 31~0 - dimy[31:0] (Read/Write)
-- 0xa4 : reserved
-- 0xa8 : Data signal of dimz
--        bit 31~0 - dimz[31:0] (Read/Write)
-- 0xac : reserved
-- 0xb0 : Data signal of rescan
--        bit 31~0 - rescan[31:0] (Read/Write)
-- 0xb4 : reserved
-- (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

architecture behave of s2m_vmap_control_s_axi is
    type states is (wridle, wrdata, wrresp, wrreset, rdidle, rddata, rdreset);  -- read and write fsm states
    signal wstate  : states := wrreset;
    signal rstate  : states := rdreset;
    signal wnext, rnext: states;
    constant ADDR_AP_CTRL       : INTEGER := 16#00#;
    constant ADDR_GIE           : INTEGER := 16#04#;
    constant ADDR_IER           : INTEGER := 16#08#;
    constant ADDR_ISR           : INTEGER := 16#0c#;
    constant ADDR_Q_DATA_0      : INTEGER := 16#10#;
    constant ADDR_Q_DATA_1      : INTEGER := 16#14#;
    constant ADDR_Q_CTRL        : INTEGER := 16#18#;
    constant ADDR_VBW_DATA_0    : INTEGER := 16#1c#;
    constant ADDR_VBW_DATA_1    : INTEGER := 16#20#;
    constant ADDR_VBW_CTRL      : INTEGER := 16#24#;
    constant ADDR_TAB_DATA_0    : INTEGER := 16#28#;
    constant ADDR_TAB_DATA_1    : INTEGER := 16#2c#;
    constant ADDR_TAB_CTRL      : INTEGER := 16#30#;
    constant ADDR_PARQ_DATA_0   : INTEGER := 16#34#;
    constant ADDR_PARQ_DATA_1   : INTEGER := 16#38#;
    constant ADDR_PARQ_CTRL     : INTEGER := 16#3c#;
    constant ADDR_PARM_DATA_0   : INTEGER := 16#40#;
    constant ADDR_PARM_DATA_1   : INTEGER := 16#44#;
    constant ADDR_PARM_CTRL     : INTEGER := 16#48#;
    constant ADDR_HB_DATA_0     : INTEGER := 16#4c#;
    constant ADDR_HB_DATA_1     : INTEGER := 16#50#;
    constant ADDR_HB_CTRL       : INTEGER := 16#54#;
    constant ADDR_NQ_DATA_0     : INTEGER := 16#58#;
    constant ADDR_NQ_CTRL       : INTEGER := 16#5c#;
    constant ADDR_MODE_DATA_0   : INTEGER := 16#60#;
    constant ADDR_MODE_CTRL     : INTEGER := 16#64#;
    constant ADDR_L_DATA_0      : INTEGER := 16#68#;
    constant ADDR_L_CTRL        : INTEGER := 16#6c#;
    constant ADDR_INVL_DATA_0   : INTEGER := 16#70#;
    constant ADDR_INVL_CTRL     : INTEGER := 16#74#;
    constant ADDR_RMAX_DATA_0   : INTEGER := 16#78#;
    constant ADDR_RMAX_CTRL     : INTEGER := 16#7c#;
    constant ADDR_ORGX_DATA_0   : INTEGER := 16#80#;
    constant ADDR_ORGX_CTRL     : INTEGER := 16#84#;
    constant ADDR_ORGY_DATA_0   : INTEGER := 16#88#;
    constant ADDR_ORGY_CTRL     : INTEGER := 16#8c#;
    constant ADDR_ORGZ_DATA_0   : INTEGER := 16#90#;
    constant ADDR_ORGZ_CTRL     : INTEGER := 16#94#;
    constant ADDR_DIMX_DATA_0   : INTEGER := 16#98#;
    constant ADDR_DIMX_CTRL     : INTEGER := 16#9c#;
    constant ADDR_DIMY_DATA_0   : INTEGER := 16#a0#;
    constant ADDR_DIMY_CTRL     : INTEGER := 16#a4#;
    constant ADDR_DIMZ_DATA_0   : INTEGER := 16#a8#;
    constant ADDR_DIMZ_CTRL     : INTEGER := 16#ac#;
    constant ADDR_RESCAN_DATA_0 : INTEGER := 16#b0#;
    constant ADDR_RESCAN_CTRL   : INTEGER := 16#b4#;
    constant ADDR_BITS         : INTEGER := 8;

    signal waddr               : UNSIGNED(ADDR_BITS-1 downto 0);
    signal wmask               : UNSIGNED(C_S_AXI_DATA_WIDTH-1 downto 0);
    signal aw_hs               : STD_LOGIC;
    signal w_hs                : STD_LOGIC;
    signal rdata_data          : UNSIGNED(C_S_AXI_DATA_WIDTH-1 downto 0);
    signal ar_hs               : STD_LOGIC;
    signal raddr               : UNSIGNED(ADDR_BITS-1 downto 0);
    signal AWREADY_t           : STD_LOGIC;
    signal WREADY_t            : STD_LOGIC;
    signal ARREADY_t           : STD_LOGIC;
    signal RVALID_t            : STD_LOGIC;
    -- internal registers
    signal int_ap_idle         : STD_LOGIC := '0';
    signal int_ap_ready        : STD_LOGIC := '0';
    signal task_ap_ready       : STD_LOGIC;
    signal int_ap_done         : STD_LOGIC := '0';
    signal task_ap_done        : STD_LOGIC;
    signal int_task_ap_done    : STD_LOGIC := '0';
    signal int_ap_start        : STD_LOGIC := '0';
    signal int_interrupt       : STD_LOGIC := '0';
    signal int_auto_restart    : STD_LOGIC := '0';
    signal auto_restart_status : STD_LOGIC := '0';
    signal auto_restart_done   : STD_LOGIC;
    signal int_gie             : STD_LOGIC := '0';
    signal int_ier             : UNSIGNED(1 downto 0) := (others => '0');
    signal int_isr             : UNSIGNED(1 downto 0) := (others => '0');
    signal int_q               : UNSIGNED(63 downto 0) := (others => '0');
    signal int_vbw             : UNSIGNED(63 downto 0) := (others => '0');
    signal int_tab             : UNSIGNED(63 downto 0) := (others => '0');
    signal int_parq            : UNSIGNED(63 downto 0) := (others => '0');
    signal int_parm            : UNSIGNED(63 downto 0) := (others => '0');
    signal int_Hb              : UNSIGNED(63 downto 0) := (others => '0');
    signal int_nq              : UNSIGNED(31 downto 0) := (others => '0');
    signal int_mode            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_L               : UNSIGNED(31 downto 0) := (others => '0');
    signal int_invL            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_rmax            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_orgx            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_orgy            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_orgz            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dimx            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dimy            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_dimz            : UNSIGNED(31 downto 0) := (others => '0');
    signal int_rescan          : UNSIGNED(31 downto 0) := (others => '0');


begin
-- ----------------------- Instantiation------------------


-- ----------------------- AXI WRITE ---------------------
    AWREADY_t <=  '1' when wstate = wridle else '0';
    AWREADY   <=  AWREADY_t;
    WREADY_t  <=  '1' when wstate = wrdata else '0';
    WREADY    <=  WREADY_t;
    BRESP     <=  "00";  -- OKAY
    BVALID    <=  '1' when wstate = wrresp else '0';
    wmask     <=  (31 downto 24 => WSTRB(3), 23 downto 16 => WSTRB(2), 15 downto 8 => WSTRB(1), 7 downto 0 => WSTRB(0));
    aw_hs     <=  AWVALID and AWREADY_t;
    w_hs      <=  WVALID and WREADY_t;

    -- write FSM
    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                wstate <= wrreset;
            elsif (ACLK_EN = '1') then
                wstate <= wnext;
            end if;
        end if;
    end process;

    process (wstate, AWVALID, WVALID, BREADY)
    begin
        case (wstate) is
        when wridle =>
            if (AWVALID = '1') then
                wnext <= wrdata;
            else
                wnext <= wridle;
            end if;
        when wrdata =>
            if (WVALID = '1') then
                wnext <= wrresp;
            else
                wnext <= wrdata;
            end if;
        when wrresp =>
            if (BREADY = '1') then
                wnext <= wridle;
            else
                wnext <= wrresp;
            end if;
        when others =>
            wnext <= wridle;
        end case;
    end process;

    waddr_proc : process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (aw_hs = '1') then
                    waddr <= UNSIGNED(AWADDR(ADDR_BITS-1 downto 2) & (1 downto 0 => '0'));
                end if;
            end if;
        end if;
    end process;

-- ----------------------- AXI READ ----------------------
    ARREADY_t <= '1' when (rstate = rdidle) else '0';
    ARREADY <= ARREADY_t;
    RDATA   <= STD_LOGIC_VECTOR(rdata_data);
    RRESP   <= "00";  -- OKAY
    RVALID_t  <= '1' when (rstate = rddata) else '0';
    RVALID    <= RVALID_t;
    ar_hs   <= ARVALID and ARREADY_t;
    raddr   <= UNSIGNED(ARADDR(ADDR_BITS-1 downto 0));

    -- read FSM
    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                rstate <= rdreset;
            elsif (ACLK_EN = '1') then
                rstate <= rnext;
            end if;
        end if;
    end process;

    process (rstate, ARVALID, RREADY, RVALID_t)
    begin
        case (rstate) is
        when rdidle =>
            if (ARVALID = '1') then
                rnext <= rddata;
            else
                rnext <= rdidle;
            end if;
        when rddata =>
            if (RREADY = '1' and RVALID_t = '1') then
                rnext <= rdidle;
            else
                rnext <= rddata;
            end if;
        when others =>
            rnext <= rdidle;
        end case;
    end process;

    rdata_proc : process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ACLK_EN = '1') then
                if (ar_hs = '1') then
                    rdata_data <= (others => '0');
                    case (TO_INTEGER(raddr)) is
                    when ADDR_AP_CTRL =>
                        rdata_data(9) <= int_interrupt;
                        rdata_data(7) <= int_auto_restart;
                        rdata_data(3) <= int_ap_ready;
                        rdata_data(2) <= int_ap_idle;
                        rdata_data(1) <= int_task_ap_done;
                        rdata_data(0) <= int_ap_start;
                    when ADDR_GIE =>
                        rdata_data(0) <= int_gie;
                    when ADDR_IER =>
                        rdata_data(1 downto 0) <= int_ier;
                    when ADDR_ISR =>
                        rdata_data(1 downto 0) <= int_isr;
                    when ADDR_Q_DATA_0 =>
                        rdata_data <= RESIZE(int_q(31 downto 0), 32);
                    when ADDR_Q_DATA_1 =>
                        rdata_data <= RESIZE(int_q(63 downto 32), 32);
                    when ADDR_VBW_DATA_0 =>
                        rdata_data <= RESIZE(int_vbw(31 downto 0), 32);
                    when ADDR_VBW_DATA_1 =>
                        rdata_data <= RESIZE(int_vbw(63 downto 32), 32);
                    when ADDR_TAB_DATA_0 =>
                        rdata_data <= RESIZE(int_tab(31 downto 0), 32);
                    when ADDR_TAB_DATA_1 =>
                        rdata_data <= RESIZE(int_tab(63 downto 32), 32);
                    when ADDR_PARQ_DATA_0 =>
                        rdata_data <= RESIZE(int_parq(31 downto 0), 32);
                    when ADDR_PARQ_DATA_1 =>
                        rdata_data <= RESIZE(int_parq(63 downto 32), 32);
                    when ADDR_PARM_DATA_0 =>
                        rdata_data <= RESIZE(int_parm(31 downto 0), 32);
                    when ADDR_PARM_DATA_1 =>
                        rdata_data <= RESIZE(int_parm(63 downto 32), 32);
                    when ADDR_HB_DATA_0 =>
                        rdata_data <= RESIZE(int_Hb(31 downto 0), 32);
                    when ADDR_HB_DATA_1 =>
                        rdata_data <= RESIZE(int_Hb(63 downto 32), 32);
                    when ADDR_NQ_DATA_0 =>
                        rdata_data <= RESIZE(int_nq(31 downto 0), 32);
                    when ADDR_MODE_DATA_0 =>
                        rdata_data <= RESIZE(int_mode(31 downto 0), 32);
                    when ADDR_L_DATA_0 =>
                        rdata_data <= RESIZE(int_L(31 downto 0), 32);
                    when ADDR_INVL_DATA_0 =>
                        rdata_data <= RESIZE(int_invL(31 downto 0), 32);
                    when ADDR_RMAX_DATA_0 =>
                        rdata_data <= RESIZE(int_rmax(31 downto 0), 32);
                    when ADDR_ORGX_DATA_0 =>
                        rdata_data <= RESIZE(int_orgx(31 downto 0), 32);
                    when ADDR_ORGY_DATA_0 =>
                        rdata_data <= RESIZE(int_orgy(31 downto 0), 32);
                    when ADDR_ORGZ_DATA_0 =>
                        rdata_data <= RESIZE(int_orgz(31 downto 0), 32);
                    when ADDR_DIMX_DATA_0 =>
                        rdata_data <= RESIZE(int_dimx(31 downto 0), 32);
                    when ADDR_DIMY_DATA_0 =>
                        rdata_data <= RESIZE(int_dimy(31 downto 0), 32);
                    when ADDR_DIMZ_DATA_0 =>
                        rdata_data <= RESIZE(int_dimz(31 downto 0), 32);
                    when ADDR_RESCAN_DATA_0 =>
                        rdata_data <= RESIZE(int_rescan(31 downto 0), 32);
                    when others =>
                        NULL;
                    end case;
                end if;
            end if;
        end if;
    end process;

-- ----------------------- Register logic ----------------
    interrupt            <= int_interrupt;
    ap_start             <= int_ap_start;
    task_ap_done         <= (ap_done and not auto_restart_status) or auto_restart_done;
    task_ap_ready        <= ap_ready and not int_auto_restart;
    auto_restart_done    <= auto_restart_status and (ap_idle and not int_ap_idle);
    q                    <= STD_LOGIC_VECTOR(int_q);
    vbw                  <= STD_LOGIC_VECTOR(int_vbw);
    tab                  <= STD_LOGIC_VECTOR(int_tab);
    parq                 <= STD_LOGIC_VECTOR(int_parq);
    parm                 <= STD_LOGIC_VECTOR(int_parm);
    Hb                   <= STD_LOGIC_VECTOR(int_Hb);
    nq                   <= STD_LOGIC_VECTOR(int_nq);
    mode                 <= STD_LOGIC_VECTOR(int_mode);
    L                    <= STD_LOGIC_VECTOR(int_L);
    invL                 <= STD_LOGIC_VECTOR(int_invL);
    rmax                 <= STD_LOGIC_VECTOR(int_rmax);
    orgx                 <= STD_LOGIC_VECTOR(int_orgx);
    orgy                 <= STD_LOGIC_VECTOR(int_orgy);
    orgz                 <= STD_LOGIC_VECTOR(int_orgz);
    dimx                 <= STD_LOGIC_VECTOR(int_dimx);
    dimy                 <= STD_LOGIC_VECTOR(int_dimy);
    dimz                 <= STD_LOGIC_VECTOR(int_dimz);
    rescan               <= STD_LOGIC_VECTOR(int_rescan);

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_interrupt <= '0';
            elsif (ACLK_EN = '1') then
                if (int_gie = '1' and (int_isr(0) or int_isr(1)) = '1') then
                    int_interrupt <= '1';
                else
                    int_interrupt <= '0';
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_start <= '0';
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_AP_CTRL and WSTRB(0) = '1' and WDATA(0) = '1') then
                    int_ap_start <= '1';
                elsif (ap_ready = '1') then
                    int_ap_start <= int_auto_restart; -- clear on handshake/auto restart
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_done <= '0';
            elsif (ACLK_EN = '1') then
                if (true) then
                    int_ap_done <= ap_done;
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_task_ap_done <= '0';
            elsif (ACLK_EN = '1') then
                if (task_ap_done = '1') then
                    int_task_ap_done <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_AP_CTRL) then
                    int_task_ap_done <= '0'; -- clear on read
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_idle <= '0';
            elsif (ACLK_EN = '1') then
                if (true) then
                    int_ap_idle <= ap_idle;
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ap_ready <= '0';
            elsif (ACLK_EN = '1') then
                if (task_ap_ready = '1') then
                    int_ap_ready <= '1';
                elsif (ar_hs = '1' and raddr = ADDR_AP_CTRL) then
                    int_ap_ready <= '0';
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_auto_restart <= '0';
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_AP_CTRL and WSTRB(0) = '1') then
                    int_auto_restart <= WDATA(7);
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                auto_restart_status <= '0';
            elsif (ACLK_EN = '1') then
                if (int_auto_restart = '1') then
                    auto_restart_status <= '1';
                elsif (ap_idle = '1') then
                    auto_restart_status <= '0';
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_gie <= '0';
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_GIE and WSTRB(0) = '1') then
                    int_gie <= WDATA(0);
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_ier <= (others=>'0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_IER and WSTRB(0) = '1') then
                    int_ier <= UNSIGNED(WDATA(1 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_isr(0) <= '0';
            elsif (ACLK_EN = '1') then
                if (int_ier(0) = '1' and ap_done = '1') then
                    int_isr(0) <= '1';
                elsif (w_hs = '1' and waddr = ADDR_ISR and WSTRB(0) = '1') then
                    int_isr(0) <= int_isr(0) xor WDATA(0); -- toggle on write
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_isr(1) <= '0';
            elsif (ACLK_EN = '1') then
                if (int_ier(1) = '1' and ap_ready = '1') then
                    int_isr(1) <= '1';
                elsif (w_hs = '1' and waddr = ADDR_ISR and WSTRB(0) = '1') then
                    int_isr(1) <= int_isr(1) xor WDATA(1); -- toggle on write
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_q(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_Q_DATA_0) then
                    int_q(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_q(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_q(63 downto 32) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_Q_DATA_1) then
                    int_q(63 downto 32) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_q(63 downto 32));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_vbw(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_VBW_DATA_0) then
                    int_vbw(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_vbw(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_vbw(63 downto 32) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_VBW_DATA_1) then
                    int_vbw(63 downto 32) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_vbw(63 downto 32));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_tab(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_TAB_DATA_0) then
                    int_tab(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_tab(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_tab(63 downto 32) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_TAB_DATA_1) then
                    int_tab(63 downto 32) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_tab(63 downto 32));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_parq(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_PARQ_DATA_0) then
                    int_parq(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_parq(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_parq(63 downto 32) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_PARQ_DATA_1) then
                    int_parq(63 downto 32) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_parq(63 downto 32));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_parm(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_PARM_DATA_0) then
                    int_parm(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_parm(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_parm(63 downto 32) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_PARM_DATA_1) then
                    int_parm(63 downto 32) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_parm(63 downto 32));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_Hb(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_HB_DATA_0) then
                    int_Hb(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_Hb(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_Hb(63 downto 32) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_HB_DATA_1) then
                    int_Hb(63 downto 32) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_Hb(63 downto 32));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_nq(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_NQ_DATA_0) then
                    int_nq(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_nq(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_mode(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_MODE_DATA_0) then
                    int_mode(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_mode(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_L(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_L_DATA_0) then
                    int_L(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_L(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_invL(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_INVL_DATA_0) then
                    int_invL(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_invL(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_rmax(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_RMAX_DATA_0) then
                    int_rmax(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_rmax(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_orgx(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_ORGX_DATA_0) then
                    int_orgx(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_orgx(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_orgy(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_ORGY_DATA_0) then
                    int_orgy(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_orgy(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_orgz(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_ORGZ_DATA_0) then
                    int_orgz(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_orgz(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dimx(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DIMX_DATA_0) then
                    int_dimx(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dimx(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dimy(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DIMY_DATA_0) then
                    int_dimy(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dimy(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_dimz(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_DIMZ_DATA_0) then
                    int_dimz(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_dimz(31 downto 0));
                end if;
            end if;
        end if;
    end process;

    process (ACLK)
    begin
        if (ACLK'event and ACLK = '1') then
            if (ARESET = '1') then
                int_rescan(31 downto 0) <= (others => '0');
            elsif (ACLK_EN = '1') then
                if (w_hs = '1' and waddr = ADDR_RESCAN_DATA_0) then
                    int_rescan(31 downto 0) <= (UNSIGNED(WDATA(31 downto 0)) and wmask(31 downto 0)) or ((not wmask(31 downto 0)) and int_rescan(31 downto 0));
                end if;
            end if;
        end if;
    end process;


-- ----------------------- Memory logic ------------------

end architecture behave;
