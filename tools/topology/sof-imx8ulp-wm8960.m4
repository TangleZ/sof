#
# Topology for i.MX8ULP board with wm8960 codec
#

# Include topology builder
include(`utils.m4')
include(`dai.m4')
include(`pipeline.m4')
include(`sai.m4')
include(`pcm.m4')
include(`buffer.m4')

# Include TLV library
include(`common/tlv.m4')

# Include Token library
include(`sof/tokens.m4')

# Include DSP configuration
include(`platform/imx/imx8qxp.m4')

#
# Define the pipelines
#
# PCM0 <----> volume <-----> SAI5 (connect to BT)
#

dnl PIPELINE_PCM_ADD(pipeline,
dnl     pipe id, pcm, max channels, format,
dnl     period, priority, core,
dnl     pcm_min_rate, pcm_max_rate, pipeline_rate,
dnl     time_domain, sched_comp)

# Low Latency playback pipeline 1 on PCM 0 using max 2 channels of s32le.
# Set 1000us deadline on core 0 with priority 0
PIPELINE_PCM_ADD(sof/pipe-volume-playback.m4,
	1, 0, 2, s32le,
	1000, 0, 0,
	48000, 48000, 48000)

# Low Latency capture pipeline 2 on PCM 0 using max 2 channels of s32le.
# Set 1000us deadline on core 0 with priority 0
PIPELINE_PCM_ADD(sof/pipe-volume-capture.m4,
	2, 0, 2, s32le,
	1000, 0, 0,
	48000, 48000, 48000)
#
# DAIs configuration
#

dnl DAI_ADD(pipeline,
dnl     pipe id, dai type, dai_index, dai_be,
dnl     buffer, periods, format,
dnl     period, priority, core, time_domain)

# playback DAI is SAI6 using 2 periods
# Buffers use s32le format, with 48 frame per 1000us on core 0 with priority 0
DAI_ADD(sof/pipe-dai-playback.m4,
	1, SAI, 5, sai5-bt-sco-pcm-wb,
	PIPELINE_SOURCE_1, 2, s32le,
	1000, 0, 0, SCHEDULE_TIME_DOMAIN_DMA)

# capture DAI is SAI6 using 2 periods
# Buffers use s32le format, with 48 frame per 1000us on core 0 with priority 0
DAI_ADD(sof/pipe-dai-capture.m4,
	2, SAI, 5, sai5-bt-sco-pcm-wb,
	PIPELINE_SINK_2, 2, s32le,
	1000, 0, 0)


# PCM Low Latency, id 0

dnl PCM_DUPLEX_ADD(name, pcm_id, playback, capture)
PCM_DUPLEX_ADD(Port0, 0, PIPELINE_PCM_1, PIPELINE_PCM_2)

dnl DAI_CONFIG(type, idx, link_id, name, sai_config)
DAI_CONFIG(SAI, 5, 0, sai5-bt-sco-pcm-wb,
	SAI_CONFIG(I2S, SAI_CLOCK(mclk, 12288000, codec_mclk_out),
		SAI_CLOCK(bclk, 3072000, codec_slave),
		SAI_CLOCK(fsync, 48000, codec_slave),
		SAI_TDM(2, 32, 3, 3),
		SAI_CONFIG_DATA(SAI, 5, 0)))
