
h
Command: %s
53*	vivadotcl27
#write_bitstream -force top_fpga.bit2default:defaultZ4-113h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
xc7z0102default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
xc7z0102default:defaultZ17-349h px? 
x
,Running DRC as a precondition to command %s
1349*	planAhead2#
write_bitstream2default:defaultZ12-1349h px? 
>
IP Catalog is up to date.1232*coregenZ19-1839h px? 
P
Running DRC with %s threads
24*drc2
42default:defaultZ23-27h px? 
?
YReport rule limit reached: REQP-1839 rule limit reached: 20 violations have been found.%s*DRC29
 !DRC|DRC System|Rule limit reached2default:default8ZCHECK-3h px? 
?
YReport rule limit reached: REQP-1840 rule limit reached: 20 violations have been found.%s*DRC29
 !DRC|DRC System|Rule limit reached2default:default8ZCHECK-3h px? 
?
fInput pipelining: DSP %s input %s is not pipelined. Pipelining DSP48 input will improve performance.%s*DRC2?
 "l
*top_digital_inst/CORE/multiplier_0/product	*top_digital_inst/CORE/multiplier_0/product2default:default2default:default2?
 "v
2top_digital_inst/CORE/multiplier_0/product/A[29:0],top_digital_inst/CORE/multiplier_0/product/A2default:default2default:default2=
 %DRC|Netlist|Instance|Pipeline|DSP48E12default:default8ZDPIP-1h px? 
?
fInput pipelining: DSP %s input %s is not pipelined. Pipelining DSP48 input will improve performance.%s*DRC2?
 "l
*top_digital_inst/CORE/multiplier_0/product	*top_digital_inst/CORE/multiplier_0/product2default:default2default:default2?
 "v
2top_digital_inst/CORE/multiplier_0/product/B[17:0],top_digital_inst/CORE/multiplier_0/product/B2default:default2default:default2=
 %DRC|Netlist|Instance|Pipeline|DSP48E12default:default8ZDPIP-1h px? 
?
?PREG Output pipelining: DSP %s output %s is not pipelined (PREG=0). Pipelining the DSP48 output will improve performance and often saves power so it is suggested whenever possible to fully pipeline this function.  If this DSP48 function was inferred, it is suggested to describe an additional register stage after this function.  If the DSP48 was instantiated in the design, it is suggested to set the PREG attribute to 1.%s*DRC2?
 "l
*top_digital_inst/CORE/multiplier_0/product	*top_digital_inst/CORE/multiplier_0/product2default:default2default:default2?
 "v
2top_digital_inst/CORE/multiplier_0/product/P[47:0],top_digital_inst/CORE/multiplier_0/product/P2default:default2default:default2=
 %DRC|Netlist|Instance|Pipeline|DSP48E12default:default8ZDPOP-1h px? 
?
?MREG Output pipelining: DSP %s multiplier stage %s is not pipelined (MREG=0). Pipelining the multiplier function will improve performance and will save significant power so it is suggested whenever possible to fully pipeline this function.  If this multiplier was inferred, it is suggested to describe an additional register stage after this function.  If there is no registered adder/accumulator following the multiply function, two pipeline stages are suggested to allow both the MREG and PREG registers to be used.  If the DSP48 was instantiated in the design, it is suggested to set both the MREG and PREG attributes to 1 when performing multiply functions.%s*DRC2?
 "l
*top_digital_inst/CORE/multiplier_0/product	*top_digital_inst/CORE/multiplier_0/product2default:default2default:default2?
 "v
2top_digital_inst/CORE/multiplier_0/product/P[47:0],top_digital_inst/CORE/multiplier_0/product/P2default:default2default:default2=
 %DRC|Netlist|Instance|Pipeline|DSP48E12default:default8ZDPOP-2h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
<top_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/CLK<top_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/CLK2default:default2default:default2?
 "?
Htop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reshi[15]_i_2/OHtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reshi[15]_i_2/O2default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reshi[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reshi[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_regDtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg2default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/op2[15]_i_2/OFtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/op2[15]_i_2/O2default:default2default:default2?
 "?
Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/op2[15]_i_2	Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/op2[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_0Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_02default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/acc_sel_i_2/OFtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/acc_sel_i_2/O2default:default2default:default2?
 "?
Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/acc_sel_i_2	Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/acc_sel_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_1Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_12default:default2default:default2?
 "?
Htop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reslo[15]_i_2/OHtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reslo[15]_i_2/O2default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reslo[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reslo[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_2Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_22default:default2default:default2?
 "?
Rtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/pmem_dout_bckup[15]_i_2/ORtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/pmem_dout_bckup[15]_i_2/O2default:default2default:default2?
 "?
Ptop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/pmem_dout_bckup[15]_i_2	Ptop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/pmem_dout_bckup[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_3Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_32default:default2default:default2?
 "?
Htop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/wdtctl[7]_i_1/OHtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/wdtctl[7]_i_1/O2default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/wdtctl[7]_i_1	Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/wdtctl[7]_i_12default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_4Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/clk_50M_reg_42default:default2default:default2?
 "?
Stop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/enable_latch_reg_i_2__23/OStop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/enable_latch_reg_i_2__23/O2default:default2default:default2?
 "?
Qtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/enable_latch_reg_i_2__23	Qtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/enable_latch_reg_i_2__232default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_1Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_12default:default2default:default2?
 "?
Ntop_digital_inst/CORE/clock_module_0/clock_gate_mclk/enable_latch_reg_i_2__1/ONtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/enable_latch_reg_i_2__1/O2default:default2default:default2?
 "?
Ltop_digital_inst/CORE/clock_module_0/clock_gate_mclk/enable_latch_reg_i_2__1	Ltop_digital_inst/CORE/clock_module_0/clock_gate_mclk/enable_latch_reg_i_2__12default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_10Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_102default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r9[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r9[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r9[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r9[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_11Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_112default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r7[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r7[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r7[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r7[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_12Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_122default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r5[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r5[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r5[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r5[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_13Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_132default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r3[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r3[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r3[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r3[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_14Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_142default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r1[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r1[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r1[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r1[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_15Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_152default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r4[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r4[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r4[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r4[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_16Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_162default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r6[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r6[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r6[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r6[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_17Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_172default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r8[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r8[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r8[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r8[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_18Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_182default:default2default:default2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r10[15]_i_2/OBtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r10[15]_i_2/O2default:default2default:default2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r10[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r10[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_19Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_192default:default2default:default2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r12[15]_i_2/OBtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r12[15]_i_2/O2default:default2default:default2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r12[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r12[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_20Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_202default:default2default:default2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r14[15]_i_2/OBtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r14[15]_i_2/O2default:default2default:default2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r14[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r14[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_21Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_212default:default2default:default2?
 "?
Itop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_in_buf[15]_i_2/OItop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_in_buf[15]_i_2/O2default:default2default:default2?
 "?
Gtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_in_buf[15]_i_2	Gtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_in_buf[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_3Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_32default:default2default:default2?
 "?
Htop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_sext[15]_i_2/OHtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_sext[15]_i_2/O2default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_sext[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_sext[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_4Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_42default:default2default:default2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/pc[15]_i_2/OAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/pc[15]_i_2/O2default:default2default:default2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/pc[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/pc[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_5Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_52default:default2default:default2?
 "?
Htop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_dext[15]_i_2/OHtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_dext[15]_i_2/O2default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_dext[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_dext[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_6Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_62default:default2default:default2?
 "?
Jtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_out_nxt[15]_i_2/OJtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_out_nxt[15]_i_2/O2default:default2default:default2?
 "?
Htop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_out_nxt[15]_i_2	Htop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_out_nxt[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_7Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_72default:default2default:default2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r15[15]_i_2/OBtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r15[15]_i_2/O2default:default2default:default2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r15[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r15[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_8Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_82default:default2default:default2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r13[15]_i_2/OBtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r13[15]_i_2/O2default:default2default:default2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r13[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r13[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_9Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/clk_50M_reg_92default:default2default:default2?
 "?
Btop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r11[15]_i_2/OBtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/r11[15]_i_2/O2default:default2default:default2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r11[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r11[15]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mclk_irq_numAtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mclk_irq_num2default:default2default:default2?
 "?
Etop_digital_inst/CORE/clock_module_0/clock_gate_mclk/irq_num[3]_i_2/OEtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/irq_num[3]_i_2/O2default:default2default:default2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/irq_num[3]_i_2	Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/irq_num[3]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
<top_digital_inst/CORE/clock_module_0/clock_gate_mclk/mclk_r2<top_digital_inst/CORE/clock_module_0/clock_gate_mclk/mclk_r22default:default2default:default2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r2[8]_i_2/O@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r2[8]_i_2/O2default:default2default:default2?
 "?
>top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r2[8]_i_2	>top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r2[8]_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_smclk/clk_50M_regAtop_digital_inst/CORE/clock_module_0/clock_gate_smclk/clk_50M_reg2default:default2default:default2?
 "?
Ptop_digital_inst/CORE/clock_module_0/clock_gate_smclk/enable_latch_reg_i_2__24/OPtop_digital_inst/CORE/clock_module_0/clock_gate_smclk/enable_latch_reg_i_2__24/O2default:default2default:default2?
 "?
Ntop_digital_inst/CORE/clock_module_0/clock_gate_smclk/enable_latch_reg_i_2__24	Ntop_digital_inst/CORE/clock_module_0/clock_gate_smclk/enable_latch_reg_i_2__242default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2?
 "?
Atop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdt_clk_cntAtop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdt_clk_cnt2default:default2default:default2?
 "?
Gtop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdtisx_s[1]_i_1/OGtop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdtisx_s[1]_i_1/O2default:default2default:default2?
 "?
Etop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdtisx_s[1]_i_1	Etop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdtisx_s[1]_i_12default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2n
 "X
 top_digital_inst/clk_divider/CLK top_digital_inst/clk_divider/CLK2default:default2default:default2?
 "|
2top_digital_inst/clk_divider/data_sync[1]_i_1__0/O2top_digital_inst/clk_divider/data_sync[1]_i_1__0/O2default:default2default:default2?
 "x
0top_digital_inst/clk_divider/data_sync[1]_i_1__0	0top_digital_inst/clk_divider/data_sync[1]_i_1__02default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2p
 "Z
!top_digital_inst/clk_divider/aclk!top_digital_inst/clk_divider/aclk2default:default2default:default2?
 "l
*top_digital_inst/clk_divider/pwm_out_i_2/O*top_digital_inst/clk_divider/pwm_out_i_2/O2default:default2default:default2~
 "h
(top_digital_inst/clk_divider/pwm_out_i_2	(top_digital_inst/clk_divider/pwm_out_i_22default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Gated clock check: Net %s is a gated clock net sourced by a combinational pin %s, cell %s. This is not good design practice and will likely impact performance. For SLICE registers, for example, use the CE pin to control the loading of data.%s*DRC2x
 "b
%top_digital_inst/clk_divider/cpu_mclk%top_digital_inst/clk_divider/cpu_mclk2default:default2default:default2?
 "v
/top_digital_inst/clk_divider/data_sync[1]_i_1/O/top_digital_inst/clk_divider/data_sync[1]_i_1/O2default:default2default:default2?
 "r
-top_digital_inst/clk_divider/data_sync[1]_i_1	-top_digital_inst/clk_divider/data_sync[1]_i_12default:default2default:default2=
 %DRC|Physical Configuration|Chip Level2default:default8ZPDRC-153h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 18 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/acc_sel_i_2	Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/acc_sel_i_22default:default2default:default2?
 "t
.top_digital_inst/CORE/multiplier_0/acc_sel_reg	.top_digital_inst/CORE/multiplier_0/acc_sel_reg2default:default"r
-top_digital_inst/CORE/multiplier_0/op1_reg[0]	-top_digital_inst/CORE/multiplier_0/op1_reg[0]2default:default"t
.top_digital_inst/CORE/multiplier_0/op1_reg[10]	.top_digital_inst/CORE/multiplier_0/op1_reg[10]2default:default"t
.top_digital_inst/CORE/multiplier_0/op1_reg[11]	.top_digital_inst/CORE/multiplier_0/op1_reg[11]2default:default"t
.top_digital_inst/CORE/multiplier_0/op1_reg[12]	.top_digital_inst/CORE/multiplier_0/op1_reg[12]2default:default"t
.top_digital_inst/CORE/multiplier_0/op1_reg[13]	.top_digital_inst/CORE/multiplier_0/op1_reg[13]2default:default"t
.top_digital_inst/CORE/multiplier_0/op1_reg[14]	.top_digital_inst/CORE/multiplier_0/op1_reg[14]2default:default"t
.top_digital_inst/CORE/multiplier_0/op1_reg[15]	.top_digital_inst/CORE/multiplier_0/op1_reg[15]2default:default"r
-top_digital_inst/CORE/multiplier_0/op1_reg[1]	-top_digital_inst/CORE/multiplier_0/op1_reg[1]2default:default"r
-top_digital_inst/CORE/multiplier_0/op1_reg[2]	-top_digital_inst/CORE/multiplier_0/op1_reg[2]2default:default"r
-top_digital_inst/CORE/multiplier_0/op1_reg[3]	-top_digital_inst/CORE/multiplier_0/op1_reg[3]2default:default"r
-top_digital_inst/CORE/multiplier_0/op1_reg[4]	-top_digital_inst/CORE/multiplier_0/op1_reg[4]2default:default"r
-top_digital_inst/CORE/multiplier_0/op1_reg[5]	-top_digital_inst/CORE/multiplier_0/op1_reg[5]2default:default"r
-top_digital_inst/CORE/multiplier_0/op1_reg[6]	-top_digital_inst/CORE/multiplier_0/op1_reg[6]2default:default"n
-top_digital_inst/CORE/multiplier_0/op1_reg[7]	-top_digital_inst/CORE/multiplier_0/op1_reg[7]2default:..."/
(the first 15 of 18 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/op2[15]_i_2	Dtop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/op2[15]_i_22default:default2default:default2?
 "r
-top_digital_inst/CORE/multiplier_0/op2_reg[0]	-top_digital_inst/CORE/multiplier_0/op2_reg[0]2default:default"t
.top_digital_inst/CORE/multiplier_0/op2_reg[10]	.top_digital_inst/CORE/multiplier_0/op2_reg[10]2default:default"t
.top_digital_inst/CORE/multiplier_0/op2_reg[11]	.top_digital_inst/CORE/multiplier_0/op2_reg[11]2default:default"t
.top_digital_inst/CORE/multiplier_0/op2_reg[12]	.top_digital_inst/CORE/multiplier_0/op2_reg[12]2default:default"t
.top_digital_inst/CORE/multiplier_0/op2_reg[13]	.top_digital_inst/CORE/multiplier_0/op2_reg[13]2default:default"t
.top_digital_inst/CORE/multiplier_0/op2_reg[14]	.top_digital_inst/CORE/multiplier_0/op2_reg[14]2default:default"t
.top_digital_inst/CORE/multiplier_0/op2_reg[15]	.top_digital_inst/CORE/multiplier_0/op2_reg[15]2default:default"r
-top_digital_inst/CORE/multiplier_0/op2_reg[1]	-top_digital_inst/CORE/multiplier_0/op2_reg[1]2default:default"r
-top_digital_inst/CORE/multiplier_0/op2_reg[2]	-top_digital_inst/CORE/multiplier_0/op2_reg[2]2default:default"r
-top_digital_inst/CORE/multiplier_0/op2_reg[3]	-top_digital_inst/CORE/multiplier_0/op2_reg[3]2default:default"r
-top_digital_inst/CORE/multiplier_0/op2_reg[4]	-top_digital_inst/CORE/multiplier_0/op2_reg[4]2default:default"r
-top_digital_inst/CORE/multiplier_0/op2_reg[5]	-top_digital_inst/CORE/multiplier_0/op2_reg[5]2default:default"r
-top_digital_inst/CORE/multiplier_0/op2_reg[6]	-top_digital_inst/CORE/multiplier_0/op2_reg[6]2default:default"r
-top_digital_inst/CORE/multiplier_0/op2_reg[7]	-top_digital_inst/CORE/multiplier_0/op2_reg[7]2default:default"n
-top_digital_inst/CORE/multiplier_0/op2_reg[8]	-top_digital_inst/CORE/multiplier_0/op2_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Ptop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/pmem_dout_bckup[15]_i_2	Ptop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/pmem_dout_bckup[15]_i_22default:default2default:default2?
 "?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[0]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[0]2default:default"?
<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[10]	<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[10]2default:default"?
<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[11]	<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[11]2default:default"?
<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[12]	<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[12]2default:default"?
<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[13]	<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[13]2default:default"?
<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[14]	<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[14]2default:default"?
<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[15]	<top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[15]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[1]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[1]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[2]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[2]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[3]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[3]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[4]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[4]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[5]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[5]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[6]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[6]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[7]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[7]2default:default"?
;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[8]	;top_digital_inst/CORE/mem_backbone_0/pmem_dout_bckup_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reshi[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reshi[15]_i_22default:default2default:default2?
 "v
/top_digital_inst/CORE/multiplier_0/reshi_reg[0]	/top_digital_inst/CORE/multiplier_0/reshi_reg[0]2default:default"x
0top_digital_inst/CORE/multiplier_0/reshi_reg[10]	0top_digital_inst/CORE/multiplier_0/reshi_reg[10]2default:default"x
0top_digital_inst/CORE/multiplier_0/reshi_reg[11]	0top_digital_inst/CORE/multiplier_0/reshi_reg[11]2default:default"x
0top_digital_inst/CORE/multiplier_0/reshi_reg[12]	0top_digital_inst/CORE/multiplier_0/reshi_reg[12]2default:default"x
0top_digital_inst/CORE/multiplier_0/reshi_reg[13]	0top_digital_inst/CORE/multiplier_0/reshi_reg[13]2default:default"x
0top_digital_inst/CORE/multiplier_0/reshi_reg[14]	0top_digital_inst/CORE/multiplier_0/reshi_reg[14]2default:default"x
0top_digital_inst/CORE/multiplier_0/reshi_reg[15]	0top_digital_inst/CORE/multiplier_0/reshi_reg[15]2default:default"v
/top_digital_inst/CORE/multiplier_0/reshi_reg[1]	/top_digital_inst/CORE/multiplier_0/reshi_reg[1]2default:default"v
/top_digital_inst/CORE/multiplier_0/reshi_reg[2]	/top_digital_inst/CORE/multiplier_0/reshi_reg[2]2default:default"v
/top_digital_inst/CORE/multiplier_0/reshi_reg[3]	/top_digital_inst/CORE/multiplier_0/reshi_reg[3]2default:default"v
/top_digital_inst/CORE/multiplier_0/reshi_reg[4]	/top_digital_inst/CORE/multiplier_0/reshi_reg[4]2default:default"v
/top_digital_inst/CORE/multiplier_0/reshi_reg[5]	/top_digital_inst/CORE/multiplier_0/reshi_reg[5]2default:default"v
/top_digital_inst/CORE/multiplier_0/reshi_reg[6]	/top_digital_inst/CORE/multiplier_0/reshi_reg[6]2default:default"v
/top_digital_inst/CORE/multiplier_0/reshi_reg[7]	/top_digital_inst/CORE/multiplier_0/reshi_reg[7]2default:default"r
/top_digital_inst/CORE/multiplier_0/reshi_reg[8]	/top_digital_inst/CORE/multiplier_0/reshi_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reslo[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/reslo[15]_i_22default:default2default:default2?
 "v
/top_digital_inst/CORE/multiplier_0/reslo_reg[0]	/top_digital_inst/CORE/multiplier_0/reslo_reg[0]2default:default"x
0top_digital_inst/CORE/multiplier_0/reslo_reg[10]	0top_digital_inst/CORE/multiplier_0/reslo_reg[10]2default:default"x
0top_digital_inst/CORE/multiplier_0/reslo_reg[11]	0top_digital_inst/CORE/multiplier_0/reslo_reg[11]2default:default"x
0top_digital_inst/CORE/multiplier_0/reslo_reg[12]	0top_digital_inst/CORE/multiplier_0/reslo_reg[12]2default:default"x
0top_digital_inst/CORE/multiplier_0/reslo_reg[13]	0top_digital_inst/CORE/multiplier_0/reslo_reg[13]2default:default"x
0top_digital_inst/CORE/multiplier_0/reslo_reg[14]	0top_digital_inst/CORE/multiplier_0/reslo_reg[14]2default:default"x
0top_digital_inst/CORE/multiplier_0/reslo_reg[15]	0top_digital_inst/CORE/multiplier_0/reslo_reg[15]2default:default"v
/top_digital_inst/CORE/multiplier_0/reslo_reg[1]	/top_digital_inst/CORE/multiplier_0/reslo_reg[1]2default:default"v
/top_digital_inst/CORE/multiplier_0/reslo_reg[2]	/top_digital_inst/CORE/multiplier_0/reslo_reg[2]2default:default"v
/top_digital_inst/CORE/multiplier_0/reslo_reg[3]	/top_digital_inst/CORE/multiplier_0/reslo_reg[3]2default:default"v
/top_digital_inst/CORE/multiplier_0/reslo_reg[4]	/top_digital_inst/CORE/multiplier_0/reslo_reg[4]2default:default"v
/top_digital_inst/CORE/multiplier_0/reslo_reg[5]	/top_digital_inst/CORE/multiplier_0/reslo_reg[5]2default:default"v
/top_digital_inst/CORE/multiplier_0/reslo_reg[6]	/top_digital_inst/CORE/multiplier_0/reslo_reg[6]2default:default"v
/top_digital_inst/CORE/multiplier_0/reslo_reg[7]	/top_digital_inst/CORE/multiplier_0/reslo_reg[7]2default:default"r
/top_digital_inst/CORE/multiplier_0/reslo_reg[8]	/top_digital_inst/CORE/multiplier_0/reslo_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 5 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/wdtctl[7]_i_1	Ftop_digital_inst/CORE/clock_module_0/clock_gate_dma_mclk/wdtctl[7]_i_12default:default2default:default2?
 "t
.top_digital_inst/CORE/watchdog_0/wdtctl_reg[0]	.top_digital_inst/CORE/watchdog_0/wdtctl_reg[0]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtctl_reg[1]	.top_digital_inst/CORE/watchdog_0/wdtctl_reg[1]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtctl_reg[4]	.top_digital_inst/CORE/watchdog_0/wdtctl_reg[4]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtctl_reg[6]	.top_digital_inst/CORE/watchdog_0/wdtctl_reg[6]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtctl_reg[7]	.top_digital_inst/CORE/watchdog_0/wdtctl_reg[7]2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_dext[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_dext[15]_i_22default:default2default:default2?
 "z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[0]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[0]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[10]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[10]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[11]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[11]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[12]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[12]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[13]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[13]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[14]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[14]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[15]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[15]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[1]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[1]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[2]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[2]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[3]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[3]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[4]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[4]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[5]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[5]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[6]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[6]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[7]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[7]2default:default"v
1top_digital_inst/CORE/frontend_0/inst_dext_reg[8]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_sext[15]_i_2	Ftop_digital_inst/CORE/clock_module_0/clock_gate_mclk/inst_sext[15]_i_22default:default2default:default2?
 "z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[0]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[0]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_sext_reg[10]	2top_digital_inst/CORE/frontend_0/inst_sext_reg[10]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_sext_reg[11]	2top_digital_inst/CORE/frontend_0/inst_sext_reg[11]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_sext_reg[12]	2top_digital_inst/CORE/frontend_0/inst_sext_reg[12]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_sext_reg[13]	2top_digital_inst/CORE/frontend_0/inst_sext_reg[13]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_sext_reg[14]	2top_digital_inst/CORE/frontend_0/inst_sext_reg[14]2default:default"|
2top_digital_inst/CORE/frontend_0/inst_sext_reg[15]	2top_digital_inst/CORE/frontend_0/inst_sext_reg[15]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[1]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[1]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[2]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[2]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[3]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[3]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[4]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[4]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[5]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[5]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[6]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[6]2default:default"z
1top_digital_inst/CORE/frontend_0/inst_sext_reg[7]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[7]2default:default"v
1top_digital_inst/CORE/frontend_0/inst_sext_reg[8]	1top_digital_inst/CORE/frontend_0/inst_sext_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 4 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/irq_num[3]_i_2	Ctop_digital_inst/CORE/clock_module_0/clock_gate_mclk/irq_num[3]_i_22default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/irq_num_reg[0]	/top_digital_inst/CORE/frontend_0/irq_num_reg[0]2default:default"v
/top_digital_inst/CORE/frontend_0/irq_num_reg[1]	/top_digital_inst/CORE/frontend_0/irq_num_reg[1]2default:default"v
/top_digital_inst/CORE/frontend_0/irq_num_reg[2]	/top_digital_inst/CORE/frontend_0/irq_num_reg[2]2default:default"v
/top_digital_inst/CORE/frontend_0/irq_num_reg[3]	/top_digital_inst/CORE/frontend_0/irq_num_reg[3]2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Gtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_in_buf[15]_i_2	Gtop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_in_buf[15]_i_22default:default2default:default2?
 "?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[0]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[0]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[10]	9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[10]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[11]	9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[11]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[12]	9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[12]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[13]	9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[13]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[14]	9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[14]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[15]	9top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[15]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[1]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[1]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[2]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[2]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[3]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[3]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[4]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[4]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[5]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[5]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[6]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[6]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[7]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[7]2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[8]	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Htop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_out_nxt[15]_i_2	Htop_digital_inst/CORE/clock_module_0/clock_gate_mclk/mdb_out_nxt[15]_i_22default:default2default:default2?
 "?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[0]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[0]2default:default"?
:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[10]	:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[10]2default:default"?
:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[11]	:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[11]2default:default"?
:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[12]	:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[12]2default:default"?
:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[13]	:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[13]2default:default"?
:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[14]	:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[14]2default:default"?
:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[15]	:top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[15]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[1]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[1]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[2]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[2]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[3]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[3]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[4]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[4]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[5]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[5]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[6]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[6]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[7]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[7]2default:default"?
9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[8]	9top_digital_inst/CORE/execution_unit_0/mdb_out_nxt_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/pc[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/pc[15]_i_22default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[0]	*top_digital_inst/CORE/frontend_0/pc_reg[0]2default:default"n
+top_digital_inst/CORE/frontend_0/pc_reg[10]	+top_digital_inst/CORE/frontend_0/pc_reg[10]2default:default"n
+top_digital_inst/CORE/frontend_0/pc_reg[11]	+top_digital_inst/CORE/frontend_0/pc_reg[11]2default:default"n
+top_digital_inst/CORE/frontend_0/pc_reg[12]	+top_digital_inst/CORE/frontend_0/pc_reg[12]2default:default"n
+top_digital_inst/CORE/frontend_0/pc_reg[13]	+top_digital_inst/CORE/frontend_0/pc_reg[13]2default:default"n
+top_digital_inst/CORE/frontend_0/pc_reg[14]	+top_digital_inst/CORE/frontend_0/pc_reg[14]2default:default"n
+top_digital_inst/CORE/frontend_0/pc_reg[15]	+top_digital_inst/CORE/frontend_0/pc_reg[15]2default:default"l
*top_digital_inst/CORE/frontend_0/pc_reg[1]	*top_digital_inst/CORE/frontend_0/pc_reg[1]2default:default"l
*top_digital_inst/CORE/frontend_0/pc_reg[2]	*top_digital_inst/CORE/frontend_0/pc_reg[2]2default:default"l
*top_digital_inst/CORE/frontend_0/pc_reg[3]	*top_digital_inst/CORE/frontend_0/pc_reg[3]2default:default"l
*top_digital_inst/CORE/frontend_0/pc_reg[4]	*top_digital_inst/CORE/frontend_0/pc_reg[4]2default:default"l
*top_digital_inst/CORE/frontend_0/pc_reg[5]	*top_digital_inst/CORE/frontend_0/pc_reg[5]2default:default"l
*top_digital_inst/CORE/frontend_0/pc_reg[6]	*top_digital_inst/CORE/frontend_0/pc_reg[6]2default:default"l
*top_digital_inst/CORE/frontend_0/pc_reg[7]	*top_digital_inst/CORE/frontend_0/pc_reg[7]2default:default"h
*top_digital_inst/CORE/frontend_0/pc_reg[8]	*top_digital_inst/CORE/frontend_0/pc_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r10[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r10[15]_i_22default:default2default:default2?
 "?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[0]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[0]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[10]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[10]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[11]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[11]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[12]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[12]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[13]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[13]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[14]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[14]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[15]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[15]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[1]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[1]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[2]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[2]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[3]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[3]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[4]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[4]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[5]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[5]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[6]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[6]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[7]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[7]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[8]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r10_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r11[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r11[15]_i_22default:default2default:default2?
 "?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[0]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[0]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[10]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[10]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[11]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[11]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[12]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[12]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[13]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[13]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[14]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[14]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[15]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[15]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[1]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[1]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[2]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[2]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[3]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[3]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[4]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[4]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[5]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[5]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[6]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[6]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[7]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[7]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[8]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r11_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r12[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r12[15]_i_22default:default2default:default2?
 "?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[0]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[0]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[10]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[10]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[11]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[11]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[12]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[12]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[13]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[13]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[14]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[14]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[15]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[15]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[1]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[1]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[2]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[2]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[3]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[3]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[4]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[4]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[5]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[5]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[6]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[6]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[7]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[7]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[8]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r12_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r13[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r13[15]_i_22default:default2default:default2?
 "?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[0]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[0]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[10]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[10]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[11]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[11]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[12]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[12]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[13]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[13]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[14]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[14]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[15]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[15]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[1]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[1]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[2]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[2]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[3]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[3]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[4]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[4]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[5]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[5]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[6]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[6]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[7]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[7]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[8]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r13_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r14[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r14[15]_i_22default:default2default:default2?
 "?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[0]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[0]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[10]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[10]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[11]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[11]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[12]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[12]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[13]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[13]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[14]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[14]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[15]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[15]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[1]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[1]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[2]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[2]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[3]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[3]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[4]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[4]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[5]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[5]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[6]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[6]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[7]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[7]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[8]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r14_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r15[15]_i_2	@top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r15[15]_i_22default:default2default:default2?
 "?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[0]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[0]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[10]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[10]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[11]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[11]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[12]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[12]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[13]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[13]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[14]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[14]2default:default"?
Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[15]	Btop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[15]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[1]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[1]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[2]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[2]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[3]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[3]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[4]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[4]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[5]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[5]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[6]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[6]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[7]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[7]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[8]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r15_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 15 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r1[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r1[15]_i_22default:default2default:default2?
 "?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[8]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[9]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r1_reg[9]2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 9 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
>top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r2[8]_i_2	>top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r2[8]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[0]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r2_reg[8]2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r3[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r3[15]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[0]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r3_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r4[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r4[15]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[0]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r4_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r5[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r5[15]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[0]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r5_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r6[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r6[15]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[0]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r6_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r7[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r7[15]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[0]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r7_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r8[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r8[15]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[0]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r8_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 16 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r9[15]_i_2	?top_digital_inst/CORE/clock_module_0/clock_gate_mclk/r9[15]_i_22default:default2default:default2?
 "?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[0]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[0]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[10]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[10]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[11]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[11]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[12]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[12]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[13]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[13]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[14]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[14]2default:default"?
Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[15]	Atop_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[15]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[1]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[1]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[2]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[2]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[3]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[3]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[4]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[4]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[5]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[5]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[6]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[6]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[7]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[7]2default:default"?
@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[8]	@top_digital_inst/CORE/execution_unit_0/register_file_0/r9_reg[8]2default:..."/
(the first 15 of 16 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 22 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "?
Etop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdtisx_s[1]_i_1	Etop_digital_inst/CORE/clock_module_0/clock_gate_smclk/wdtisx_s[1]_i_12default:default2default:default2?
 "~
3top_digital_inst/CORE/watchdog_0/wdt_evt_toggle_reg	3top_digital_inst/CORE/watchdog_0/wdt_evt_toggle_reg2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[0]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[0]2default:default"v
/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[10]	/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[10]2default:default"v
/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[11]	/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[11]2default:default"v
/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[12]	/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[12]2default:default"v
/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[13]	/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[13]2default:default"v
/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[14]	/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[14]2default:default"v
/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[15]	/top_digital_inst/CORE/watchdog_0/wdtcnt_reg[15]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[1]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[1]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[2]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[2]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[3]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[3]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[4]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[4]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[5]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[5]2default:default"t
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[6]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[6]2default:default"p
.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[7]	.top_digital_inst/CORE/watchdog_0/wdtcnt_reg[7]2default:..."/
(the first 15 of 22 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 23 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "r
-top_digital_inst/clk_divider/data_sync[1]_i_1	-top_digital_inst/clk_divider/data_sync[1]_i_12default:default2default:default2?
 "?
7top_digital_inst/CORE/clock_module_0/dbg_rst_noscan_reg	7top_digital_inst/CORE/clock_module_0/dbg_rst_noscan_reg2default:default"?
Ftop_digital_inst/CORE/clock_module_0/sync_cell_dbg_en/data_sync_reg[0]	Ftop_digital_inst/CORE/clock_module_0/sync_cell_dbg_en/data_sync_reg[0]2default:default"?
Ftop_digital_inst/CORE/clock_module_0/sync_cell_dbg_en/data_sync_reg[1]	Ftop_digital_inst/CORE/clock_module_0/sync_cell_dbg_en/data_sync_reg[1]2default:default"?
Ctop_digital_inst/CORE/clock_module_0/sync_cell_puc/data_sync_reg[0]	Ctop_digital_inst/CORE/clock_module_0/sync_cell_puc/data_sync_reg[0]2default:default"?
Ctop_digital_inst/CORE/clock_module_0/sync_cell_puc/data_sync_reg[1]	Ctop_digital_inst/CORE/clock_module_0/sync_cell_puc/data_sync_reg[1]2default:default"|
2top_digital_inst/CORE/execution_unit_0/mab_lsb_reg	2top_digital_inst/CORE/execution_unit_0/mab_lsb_reg2default:default"?
8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_en_reg	8top_digital_inst/CORE/execution_unit_0/mdb_in_buf_en_reg2default:default"?
;top_digital_inst/CORE/execution_unit_0/mdb_in_buf_valid_reg	;top_digital_inst/CORE/execution_unit_0/mdb_in_buf_valid_reg2default:default"?
>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[0]	>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[0]2default:default"?
>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[1]	>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[1]2default:default"?
>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[2]	>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[2]2default:default"?
>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[3]	>top_digital_inst/CORE/frontend_0/FSM_sequential_e_state_reg[3]2default:default"?
>top_digital_inst/CORE/frontend_0/FSM_sequential_i_state_reg[0]	>top_digital_inst/CORE/frontend_0/FSM_sequential_i_state_reg[0]2default:default"?
>top_digital_inst/CORE/frontend_0/FSM_sequential_i_state_reg[1]	>top_digital_inst/CORE/frontend_0/FSM_sequential_i_state_reg[1]2default:default"?
>top_digital_inst/CORE/frontend_0/FSM_sequential_i_state_reg[2]	>top_digital_inst/CORE/frontend_0/FSM_sequential_i_state_reg[2]2default:..."/
(the first 15 of 23 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 7 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2?
 "x
0top_digital_inst/clk_divider/data_sync[1]_i_1__0	0top_digital_inst/clk_divider/data_sync[1]_i_1__02default:default2default:default2?
 "?
Ftop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_clr/data_sync_reg[0]	Ftop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_clr/data_sync_reg[0]2default:default"?
Ftop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_clr/data_sync_reg[1]	Ftop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_clr/data_sync_reg[1]2default:default"?
Gtop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_incr/data_sync_reg[0]	Gtop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_incr/data_sync_reg[0]2default:default"?
Gtop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_incr/data_sync_reg[1]	Gtop_digital_inst/CORE/watchdog_0/sync_cell_wdtcnt_incr/data_sync_reg[1]2default:default"?
@top_digital_inst/CORE/watchdog_0/sync_reset_por/data_sync_reg[0]	@top_digital_inst/CORE/watchdog_0/sync_reset_por/data_sync_reg[0]2default:default"?
@top_digital_inst/CORE/watchdog_0/sync_reset_por/data_sync_reg[1]	@top_digital_inst/CORE/watchdog_0/sync_reset_por/data_sync_reg[1]2default:default"?
8top_digital_inst/CORE/watchdog_0/wdtcnt_clr_sync_dly_reg	8top_digital_inst/CORE/watchdog_0/wdtcnt_clr_sync_dly_reg2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?Non-Optimal connections which could lead to hold violations: A LUT %s is driving clock pin of 17 cells. This could lead to large hold time violations. Involved cells are:
%s%s*DRC2~
 "h
(top_digital_inst/clk_divider/pwm_out_i_2	(top_digital_inst/clk_divider/pwm_out_i_22default:default2default:default2?
 "^
#top_digital_inst/PWM/counter_reg[0]	#top_digital_inst/PWM/counter_reg[0]2default:default"`
$top_digital_inst/PWM/counter_reg[10]	$top_digital_inst/PWM/counter_reg[10]2default:default"`
$top_digital_inst/PWM/counter_reg[11]	$top_digital_inst/PWM/counter_reg[11]2default:default"`
$top_digital_inst/PWM/counter_reg[12]	$top_digital_inst/PWM/counter_reg[12]2default:default"`
$top_digital_inst/PWM/counter_reg[13]	$top_digital_inst/PWM/counter_reg[13]2default:default"`
$top_digital_inst/PWM/counter_reg[14]	$top_digital_inst/PWM/counter_reg[14]2default:default"`
$top_digital_inst/PWM/counter_reg[15]	$top_digital_inst/PWM/counter_reg[15]2default:default"^
#top_digital_inst/PWM/counter_reg[1]	#top_digital_inst/PWM/counter_reg[1]2default:default"^
#top_digital_inst/PWM/counter_reg[2]	#top_digital_inst/PWM/counter_reg[2]2default:default"^
#top_digital_inst/PWM/counter_reg[3]	#top_digital_inst/PWM/counter_reg[3]2default:default"^
#top_digital_inst/PWM/counter_reg[4]	#top_digital_inst/PWM/counter_reg[4]2default:default"^
#top_digital_inst/PWM/counter_reg[5]	#top_digital_inst/PWM/counter_reg[5]2default:default"^
#top_digital_inst/PWM/counter_reg[6]	#top_digital_inst/PWM/counter_reg[6]2default:default"^
#top_digital_inst/PWM/counter_reg[7]	#top_digital_inst/PWM/counter_reg[7]2default:default"Z
#top_digital_inst/PWM/counter_reg[8]	#top_digital_inst/PWM/counter_reg[8]2default:..."/
(the first 15 of 17 listed)2default:default2default:default2A
 )DRC|Implementation|Placement|DesignChecks2default:default8ZPLHOLDVIO-2h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "p
,top_digital_inst/CORE/dbg_0/mem_data_reg[14]	,top_digital_inst/CORE/dbg_0/mem_data_reg[14]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "p
,top_digital_inst/CORE/dbg_0/mem_data_reg[15]	,top_digital_inst/CORE/dbg_0/mem_data_reg[15]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "n
+top_digital_inst/CORE/dbg_0/mem_data_reg[5]	+top_digital_inst/CORE/dbg_0/mem_data_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[0]	/top_digital_inst/CORE/frontend_0/inst_as_reg[0]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[2]	/top_digital_inst/CORE/frontend_0/inst_as_reg[2]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[4]	/top_digital_inst/CORE/frontend_0/inst_as_reg[4]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[5]	/top_digital_inst/CORE/frontend_0/inst_as_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[6]	/top_digital_inst/CORE/frontend_0/inst_as_reg[6]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[7]	/top_digital_inst/CORE/frontend_0/inst_as_reg[7]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "p
,top_digital_inst/CORE/frontend_0/inst_bw_reg	,top_digital_inst/CORE/frontend_0/inst_bw_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "?
5top_digital_inst/CORE/frontend_0/inst_dest_bin_reg[2]	5top_digital_inst/CORE/frontend_0/inst_dest_bin_reg[2]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "?
5top_digital_inst/CORE/frontend_0/inst_dest_bin_reg[3]	5top_digital_inst/CORE/frontend_0/inst_dest_bin_reg[3]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "z
1top_digital_inst/CORE/frontend_0/inst_dext_reg[0]	1top_digital_inst/CORE/frontend_0/inst_dext_reg[0]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[10]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[10]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "|
2top_digital_inst/CORE/frontend_0/inst_dext_reg[11]	2top_digital_inst/CORE/frontend_0/inst_dext_reg[11]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "n
+top_digital_inst/CORE/frontend_0/pc_reg[15]	+top_digital_inst/CORE/frontend_0/pc_reg[15]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[2]	*top_digital_inst/CORE/frontend_0/pc_reg[2]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[4]	*top_digital_inst/CORE/frontend_0/pc_reg[4]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[5]	*top_digital_inst/CORE/frontend_0/pc_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?
?RAMB36 async control check: The RAMB36E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2n
 "X
 top_digital_inst/DATAMEM/mem_reg	 top_digital_inst/DATAMEM/mem_reg2default:default2default:default2?
 "x
0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]0top_digital_inst/DATAMEM/mem_reg/ADDRARDADDR[14]2default:default2default:default2h
 "R
top_digital_inst/DATAMEM/D[9]top_digital_inst/DATAMEM/D[9]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[9]	*top_digital_inst/CORE/frontend_0/pc_reg[9]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB36E12default:default8Z	REQP-1839h px? 
?	
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "?
Ftop_digital_inst/CORE/clock_module_0/sync_cell_cpu_en/data_sync_reg[1]	Ftop_digital_inst/CORE/clock_module_0/sync_cell_cpu_en/data_sync_reg[1]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "?
6top_digital_inst/CORE/dbg_0/dbg_uart_0/dbg_addr_reg[5]	6top_digital_inst/CORE/dbg_0/dbg_uart_0/dbg_addr_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "p
,top_digital_inst/CORE/dbg_0/mem_data_reg[14]	,top_digital_inst/CORE/dbg_0/mem_data_reg[14]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "p
,top_digital_inst/CORE/dbg_0/mem_data_reg[15]	,top_digital_inst/CORE/dbg_0/mem_data_reg[15]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "n
+top_digital_inst/CORE/dbg_0/mem_data_reg[5]	+top_digital_inst/CORE/dbg_0/mem_data_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "j
)top_digital_inst/CORE/dbg_0/mem_start_reg	)top_digital_inst/CORE/dbg_0/mem_start_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "l
*top_digital_inst/CORE/dbg_0/mem_startb_reg	*top_digital_inst/CORE/dbg_0/mem_startb_reg2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "x
0top_digital_inst/CORE/frontend_0/inst_alu_reg[4]	0top_digital_inst/CORE/frontend_0/inst_alu_reg[4]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "x
0top_digital_inst/CORE/frontend_0/inst_alu_reg[5]	0top_digital_inst/CORE/frontend_0/inst_alu_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "x
0top_digital_inst/CORE/frontend_0/inst_alu_reg[7]	0top_digital_inst/CORE/frontend_0/inst_alu_reg[7]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[0]	/top_digital_inst/CORE/frontend_0/inst_as_reg[0]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[2]	/top_digital_inst/CORE/frontend_0/inst_as_reg[2]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[4]	/top_digital_inst/CORE/frontend_0/inst_as_reg[4]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "v
/top_digital_inst/CORE/frontend_0/inst_as_reg[5]	/top_digital_inst/CORE/frontend_0/inst_as_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "n
+top_digital_inst/CORE/frontend_0/pc_reg[15]	+top_digital_inst/CORE/frontend_0/pc_reg[15]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[2]	*top_digital_inst/CORE/frontend_0/pc_reg[2]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[4]	*top_digital_inst/CORE/frontend_0/pc_reg[4]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[5]	*top_digital_inst/CORE/frontend_0/pc_reg[5]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2?
 "l
*top_digital_inst/CORE/frontend_0/pc_reg[9]	*top_digital_inst/CORE/frontend_0/pc_reg[9]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
?RAMB18 async control check: The RAMB18E1 %s has an input control pin %s (net: %s) which is driven by a register (%s) that has an active asychronous set or reset. This may cause corruption of the memory contents and/or read values when the set/reset is asserted and is not analyzed by the default static timing analysis. It is suggested to eliminate the use of a set/reset to registers driving this RAMB pin or else use a synchronous reset in which the assertion of the reset is timed by default.%s*DRC2r
 "\
"top_digital_inst/PROGMEM/mem_reg_5	"top_digital_inst/PROGMEM/mem_reg_52default:default2default:default2?
 "|
2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2top_digital_inst/PROGMEM/mem_reg_5/ADDRARDADDR[13]2default:default2default:default2~
 "h
(top_digital_inst/PROGMEM/ADDRARDADDR[12](top_digital_inst/PROGMEM/ADDRARDADDR[12]2default:default2default:default2r
 "\
"top_digital_inst/GPIO/p1ifg_reg[7]	"top_digital_inst/GPIO/p1ifg_reg[7]2default:default2default:default2B
 *DRC|Netlist|Instance|Required Pin|RAMB18E12default:default8Z	REQP-1840h px? 
?
uPS7 block required: The PS7 cell must be used in this Zynq design in order to enable correct default configuration.%s*DRC2;
 #DRC|PS7|Zynq requires PS7 block|PS72default:default8ZZPS7-1h px? 
h
DRC finished with %s
1905*	planAhead2*
0 Errors, 112 Warnings2default:defaultZ12-3199h px? 
i
BPlease refer to the DRC report (report_drc) for more information.
1906*	planAheadZ12-3200h px? 
i
)Running write_bitstream with %s threads.
1750*designutils2
42default:defaultZ20-2272h px? 
?
Loading data files...
1271*designutilsZ12-1165h px? 
>
Loading site data...
1273*designutilsZ12-1167h px? 
?
Loading route data...
1272*designutilsZ12-1166h px? 
?
Processing options...
1362*designutilsZ12-1514h px? 
<
Creating bitmap...
1249*designutilsZ12-1141h px? 
7
Creating bitstream...
7*	bitstreamZ40-7h px? 
_
Writing bitstream %s...
11*	bitstream2"
./top_fpga.bit2default:defaultZ40-11h px? 
F
Bitgen Completed Successfully.
1606*	planAheadZ12-1842h px? 
?
?WebTalk data collection is mandatory when using a WebPACK part without a full Vivado license. To see the specific WebTalk data collected for your design, open the usage_statistics_webtalk.html or usage_statistics_webtalk.xml file in the implementation directory.
120*projectZ1-120h px? 
?
?'%s' has been successfully sent to Xilinx on %s. For additional details about this file, please refer to the Webtalk help file at %s.
186*common2j
V/home/iisclap/Downloads/FPGA_10NOV/FPGA_10NOV.runs/impl_2/usage_statistics_webtalk.xml2default:default2,
Thu May 20 22:49:11 20212default:default2M
9/tools/Xilinx/Vivado/2020.1/doc/webtalk_introduction.html2default:defaultZ17-186h px? 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1262default:default2
1862default:default2
02default:default2
02default:defaultZ4-41h px? 
a
%s completed successfully
29*	vivadotcl2#
write_bitstream2default:defaultZ4-42h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2%
write_bitstream: 2default:default2
00:00:332default:default2
00:00:272default:default2
3282.0472default:default2
337.2542default:default2
4752default:default2
32082default:defaultZ17-722h px? 


End Record